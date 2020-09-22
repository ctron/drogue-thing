#![no_std]
#![no_main]

mod data;

use core::fmt::Write;
use core::str::FromStr;

use cortex_m::peripheral::DWT;

use drogue_esp8266::protocol::WiFiMode;
use drogue_esp8266::{adapter::Adapter, ingress::Ingress, initialize, protocol::Response};
use drogue_network::{IpAddr, Mode, SocketAddr, TcpStack};
use drogue_tls::entropy::StaticEntropySource;
use drogue_tls::net::tcp_stack::SslTcpStack;
use drogue_tls::platform::SslPlatform;
use drogue_tls::ssl::config::{Preset, Transport, Verify};

use embedded_hal::digital::v2::OutputPin;
use heapless::{
    consts::{U128, U16, U2, U512},
    i,
    spsc::Queue,
    String,
};
use log::{info, LevelFilter};

use panic_rtt_target as _;

use rtic::app;
use rtic::cyccnt::U32Ext as RticU32Ext;
use rtt_logger::RTTLogger;
use rtt_target::rtt_init_print;

use stm32f7 as _;
use stm32f7xx_hal::gpio::gpioc::PC12;
use stm32f7xx_hal::gpio::gpiod::PD2;
use stm32f7xx_hal::gpio::{Alternate, GpioExt, AF8};
use stm32f7xx_hal::pac::UART5;
use stm32f7xx_hal::rcc::{Clocks, RccExt};
use stm32f7xx_hal::serial::{self, Rx, Serial, Tx};
use stm32f7xx_hal::time::U32Ext as StmU32Ext;

const DIGEST_DELAY: u32 = 100;

static LOGGER: RTTLogger = RTTLogger::new(LevelFilter::Info);

trait SerialConfig<Instance> {
    type TxPin;
    type RxPin;
    type Pins;
}

impl<Instance> SerialConfig<Instance> for UART5 {
    type TxPin = PC12<Alternate<AF8>>;
    type RxPin = PD2<Alternate<AF8>>;
    type Pins = (Self::TxPin, Self::RxPin);
}

type SerialInstance = UART5;
type SerialTx = Tx<SerialInstance>;
type SerialRx = Rx<SerialInstance>;
type SerialEsp = Serial<SerialInstance, <SerialInstance as SerialConfig<SerialInstance>>::Pins>;
type ESPAdapter = Adapter<'static, SerialTx>;

fn create_serial(
    clocks: Clocks,
    instance: SerialInstance,
    rx_pin: <SerialInstance as SerialConfig<SerialInstance>>::RxPin,
    tx_pin: <SerialInstance as SerialConfig<SerialInstance>>::TxPin,
) -> SerialEsp {
    Serial::new(
        instance,
        (tx_pin, rx_pin),
        clocks,
        serial::Config {
            baud_rate: 115_200.bps(),
            oversampling: serial::Oversampling::By16,
            character_match: None,
        },
    )
}

#[app(device = stm32f7xx_hal::pac, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        adapter: Option<ESPAdapter>,
        ingress: Ingress<'static, SerialRx>,
    }

    #[init(spawn = [digest])]
    fn init(mut ctx: init::Context) -> init::LateResources {
        rtt_init_print!(BlockIfFull, 2048);
        log::set_logger(&LOGGER).unwrap();
        log::set_max_level(log::LevelFilter::Trace);

        // // Enable CYCNT
        // Initialize (enable) the monotonic timer (CYCCNT)
        ctx.core.DCB.enable_trace();
        // required on Cortex-M7 devices that software lock the DWT (e.g. STM32F7)
        DWT::unlock();
        ctx.core.DWT.enable_cycle_counter();
        let device = ctx.device;

        let rcc = device.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(216.mhz()).freeze();

        let gpioc = device.GPIOC.split();
        let gpiod = device.GPIOD.split();
        let gpiog = device.GPIOG.split();

        let mut en = gpiod.pd3.into_push_pull_output();
        // reset pin
        let mut reset = gpiog.pg14.into_push_pull_output();

        let mut esp_gpio0 = gpiog.pg13.into_push_pull_output();
        let mut esp_gpio2 = gpiod.pd6.into_push_pull_output();

        // set both to HIGH (boot from flash, non-programming mode)
        esp_gpio0.set_high().unwrap();
        esp_gpio2.set_high().unwrap();

        let tx_pin = gpioc.pc12.into_alternate_af8();
        let rx_pin = gpiod.pd2.into_alternate_af8();
        let mut serial = create_serial(clocks, device.UART5, rx_pin, tx_pin);

        serial.listen(serial::Event::Rxne);
        let (tx, rx) = serial.split();

        static mut RESPONSE_QUEUE: Queue<Response, U2> = Queue(i::Queue::new());
        static mut NOTIFICATION_QUEUE: Queue<Response, U16> = Queue(i::Queue::new());

        let (adapter, ingress) = initialize(
            tx,
            rx,
            &mut en,
            &mut reset,
            unsafe { &mut RESPONSE_QUEUE },
            unsafe { &mut NOTIFICATION_QUEUE },
        )
        .unwrap();

        ctx.spawn.digest().unwrap();

        info!("initialized");

        init::LateResources {
            adapter: Some(adapter),
            ingress,
        }
    }

    #[task(schedule = [digest], priority = 2, resources = [ingress])]
    fn digest(mut ctx: digest::Context) {
        ctx.resources.ingress.lock(|ingress| ingress.digest());
        ctx.schedule
            .digest(ctx.scheduled + (DIGEST_DELAY * 100_000).cycles())
            .unwrap();
    }

    #[task(binds = UART5, priority = 10, resources = [ingress])]
    fn usart(ctx: usart::Context) {
        if let Err(b) = ctx.resources.ingress.isr() {
            info!("failed to ingress {}", b as char);
        }
    }

    #[idle(resources = [adapter])]
    fn idle(ctx: idle::Context) -> ! {
        info!("idle");

        let mut adapter = ctx.resources.adapter.take().unwrap();

        let result = adapter.get_firmware_info();
        info!("firmware: {:?}", result);

        let result = adapter.set_mode(WiFiMode::Station);
        info!("set mode {:?}", result);

        let result = adapter.join("muenchen.freifunk.net/muc_ost", "");
        info!("joined wifi {:?}", result);

        let result = adapter.get_ip_address();
        info!("IP {:?}", result);

        let network = adapter.into_network_stack();
        info!("network initialized");

        let mut ssl_platform =
            SslPlatform::setup(cortex_m_rt::heap_start() as usize, 1024 * 64).unwrap();

        ssl_platform
            .entropy_context_mut()
            .add_source(StaticEntropySource);

        ssl_platform.seed_rng().unwrap();

        let mut ssl_config = ssl_platform
            .new_client_config(Transport::Stream, Preset::Default)
            .unwrap();
        ssl_config.authmode(Verify::None);

        // consume the config, take a non-mutable ref to the underlying network.
        let secure_network = SslTcpStack::new(ssl_config, &network);

        let socket = secure_network.open(Mode::Blocking).unwrap();
        let socket_addr = SocketAddr::new(IpAddr::from_str("95.216.224.167").unwrap(), 443);

        let mut socket = secure_network.connect(socket, socket_addr).unwrap();

        info!("socket connected {:?}", result);

        /*
        let result = secure_network
            .write(&mut socket, b"GET / HTTP/1.1\r\nHost: http-endpoint-drogue-iot.apps.wonderful.iot-playground.org\r\n\r\n")
            .unwrap();
         */

        let publish = data::measure().unwrap();
        let data: String<U128> = serde_json_core::to_string(&publish).unwrap();

        let mut buffer: String<U512> = String::new();
        buffer.push_str("POST /publish/telemetry HTTP/1.1\r\nHost: http-endpoint-drogue-iot.apps.wonderful.iot-playground.org\r\n").unwrap();
        write!(buffer, "Content-Length: {}\r\n", data.len()).unwrap();
        buffer.push_str("\r\n").unwrap();
        buffer.push_str(&data).unwrap();

        info!("Request: {}", &buffer);

        let result = secure_network
            .write(&mut socket, buffer.as_bytes())
            .unwrap();

        /*
                let socket_addr = SocketAddr::new(IpAddr::from_str("192.168.1.245").unwrap(), 80);

                let mut socket = network.connect(socket, socket_addr).unwrap();

                info!("socket connected {:?}", result);

                let result = network
                    .write(&mut socket, b"GET / HTTP/1.1\r\nhost:192.168.1.245\r\n\r\n")
                    .unwrap();
        */
        info!("sent request {:?}", result);

        let network = secure_network;

        loop {
            let mut buffer = [0; 128];
            let result = network.read(&mut socket, &mut buffer);
            match result {
                Ok(len) => {
                    if len > 0 {
                        let s = core::str::from_utf8(&buffer[0..len]);
                        match s {
                            Ok(s) => {
                                info!("recv: {} ", s);
                            }
                            Err(_) => {
                                info!("recv: {} bytes (not utf8)", len);
                            }
                        }
                    }
                }
                Err(e) => {
                    info!("ERR: {:?}", e);
                    break;
                }
            }
        }

        loop {
            continue;
        }
    }

    // spare interrupt used for scheduling software tasks
    extern "C" {
        fn SPI1();
        fn SPI2();
    }
};
