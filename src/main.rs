#![no_std]
#![no_main]

mod data;
#[cfg(feature = "display")]
mod display;
mod error;

use panic_rtt_target as _;

use core::str::from_utf8;

use heapless::{consts, i, spsc::Queue, String};

use drogue_bme680::{
    Address, Bme680Controller, Bme680Sensor, Configuration, DelayWrapper, StaticProvider,
};

use drogue_embedded_timer::{MillisecondsClock10, MillisecondsTicker10};
use drogue_esp8266::{
    adapter::Adapter, ingress::Ingress, initialize, network::Esp8266IpNetworkDriver,
    protocol::Response, protocol::WiFiMode, BUFFER_LEN,
};
use drogue_http_client::{tcp::TcpSocketSinkSource, BufferResponseHandler, HttpConnection, Source};
use drogue_network::{
    addr::HostSocketAddr,
    dns::{AddrType, Dns},
    tcp::{Mode, TcpStack},
};
use drogue_tls::{
    entropy::StaticEntropySource,
    net::tcp_stack::SslTcpStack,
    platform::SslPlatform,
    ssl::config::{Preset, Transport, Verify},
};

use log::{info, warn, LevelFilter};

use rtic::app;
use rtic::cyccnt::U32Ext as _;

#[cfg(all(feature = "stm32f4xx", feature = "mpu"))]
use cortex_mpu::{
    cortex_m4::{MemoryAttributes, Region},
    AccessPermission, ArrayVec,
};

use cortex_m_rt::exception;

use embedded_hal::timer::Periodic;

#[cfg(feature = "wait")]
use embedded_hal::digital::v2::InputPin;

use rtt_logger::RTTLogger;
use rtt_target::rtt_init_print;

#[cfg(feature = "stm32f4xx")]
use stm32f4 as _;
#[cfg(feature = "stm32f4xx")]
use stm32f4xx_hal as hal;
#[cfg(feature = "stm32f4xx")]
use stm32f4xx_hal::stm32 as device;

#[cfg(feature = "stm32f7xx")]
use cortex_m::peripheral::DWT;
#[cfg(feature = "stm32f7xx")]
use stm32f7 as _;
#[cfg(feature = "stm32f7xx")]
use stm32f7xx_hal as hal;
#[cfg(feature = "stm32f7xx")]
use stm32f7xx_hal::pac as device;

use hal::{
    gpio::{Alternate, GpioExt},
    rcc::{Clocks, RccExt},
    serial::{self, Rx, Serial, Tx},
    time::U32Ext,
    timer::Event,
};

#[cfg(feature = "stm32f4xx")]
use stm32f4xx_hal::i2c::I2c;

#[cfg(feature = "stm32f411")]
use stm32f4::stm32f411::{I2C1, USART1, USART6};
#[cfg(feature = "stm32f411")]
use stm32f4xx_hal::gpio::{
    gpioa::{PA10, PA9},
    gpiob::{PB8, PB9},
    gpioc::{PC6, PC7},
    AlternateOD, AF4, AF7, AF8,
};
#[cfg(all(feature = "stm32f411", feature = "display"))]
use stm32f4xx_hal::{
    gpio::{
        gpioa::{PA5, PA6, PA7},
        gpioc::{PC4, PC5},
        AF5,
    },
    spi::Spi,
};

#[cfg(all(feature = "stm32f411", feature = "display"))]
use stm32f4::stm32f411::SPI1;

#[cfg(feature = "display")]
use embedded_hal::digital::v2::OutputPin;
#[cfg(feature = "display")]
use embedded_hal::spi::MODE_0;
#[cfg(feature = "display")]
use hal::gpio::{Output, PushPull};

#[cfg(feature = "stm32f7xx")]
use stm32f7xx_hal::i2c::{self, BlockingI2c};

#[cfg(feature = "stm32f723")]
use stm32f7::stm32f7x3::I2C2;
#[cfg(feature = "stm32f723")]
use stm32f7xx_hal::{
    gpio::{
        gpioc::PC12,
        gpiod::PD2,
        gpioh::{PH4, PH5},
        AF4, AF8,
    },
    pac::UART5,
};

#[cfg(feature = "display")]
use crate::display::{Display, State};
use crate::{data::Data, error::ThingError};

use drogue_network::addr::Ipv4Addr;

use embedded_time::{
    duration::{Extensions, Seconds},
    rate::Hertz,
    Clock, Instant,
};

#[cfg(feature = "mpu")]
use cortex_mpu::{
    cortex_m4::{CachePolicy, Mpu},
    Size, Subregions,
};

use cortex_m_rt::heap_start;

use core::convert::TryInto;
#[cfg(feature = "bsec")]
use drogue_bsec::{self as bsec, Inputs, Outputs, SampleRate};
use embedded_time::duration::Milliseconds;

const DIGEST_DELAY: u32 = 100;

#[cfg(feature = "publish")]
const ENDPOINT: &str = "http-endpoint-drogue-iot.apps.wonderful.iot-playground.org";
#[cfg(feature = "publish")]
const ENDPOINT_PORT: u16 = 443;

const LOGGER: RTTLogger = RTTLogger::new(LevelFilter::Info);

const SSL_HEAP_SIZE: usize = 48 * 1024;

// Timer
#[cfg(feature = "stm32f411")]
type TIM = stm32f4::stm32f411::TIM3;
#[cfg(feature = "stm32f723")]
type TIM = hal::pac::TIM2;
type Timer = hal::timer::Timer<TIM>;

// I2C types

pub trait I2cConfig<Instance> {
    type SclPin;
    type SdaPin;
}

#[cfg(feature = "stm32f411")]
type I2CInstance = I2C1;

#[cfg(feature = "stm32f411")]
impl<Instance> I2cConfig<Instance> for I2C1 {
    type SclPin = PB8<AlternateOD<AF4>>;
    type SdaPin = PB9<AlternateOD<AF4>>;
}

#[cfg(feature = "stm32f411")]
type I2cBmeInstance = hal::i2c::I2c<
    I2CInstance,
    (
        <I2CInstance as I2cConfig<I2CInstance>>::SclPin,
        <I2CInstance as I2cConfig<I2CInstance>>::SdaPin,
    ),
>;

#[cfg(feature = "stm32f723")]
type I2CInstance = I2C2;

#[cfg(feature = "stm32f723")]
impl<Instance> I2cConfig<Instance> for I2C2 {
    type SclPin = PH4<Alternate<AF4>>;
    type SdaPin = PH5<Alternate<AF4>>;
}

#[cfg(feature = "stm32f723")]
type I2cBmeInstance = hal::i2c::BlockingI2c<
    I2CInstance,
    <I2CInstance as I2cConfig<I2CInstance>>::SclPin,
    <I2CInstance as I2cConfig<I2CInstance>>::SdaPin,
>;

// serial types

trait SerialConfig<Instance> {
    type TxPin;
    type RxPin;
    type Pins;
}

#[cfg(feature = "stm32f411")]
type SerialInstance = USART1;

#[cfg(feature = "stm32f411")]
impl<Instance> SerialConfig<Instance> for USART1 {
    type TxPin = PA9<Alternate<AF7>>;
    type RxPin = PA10<Alternate<AF7>>;
    type Pins = (Self::TxPin, Self::RxPin);
}

#[cfg(feature = "stm32f411")]
impl<Instance> SerialConfig<Instance> for USART6 {
    type TxPin = PC6<Alternate<AF8>>;
    type RxPin = PC7<Alternate<AF8>>;
    type Pins = (Self::TxPin, Self::RxPin);
}

#[cfg(feature = "stm32f723")]
type SerialInstance = UART5;

#[cfg(feature = "stm32f723")]
impl<Instance> SerialConfig<Instance> for UART5 {
    type TxPin = PC12<Alternate<AF8>>;
    type RxPin = PD2<Alternate<AF8>>;
    type Pins = (Self::TxPin, Self::RxPin);
}

type SerialTx = Tx<SerialInstance>;
type SerialRx = Rx<SerialInstance>;
type SerialEsp = Serial<SerialInstance, <SerialInstance as SerialConfig<SerialInstance>>::Pins>;
type ESPAdapter = Adapter<'static, SerialTx>;

type ClockType = MillisecondsClock10;

static CLOCK: ClockType = MillisecondsClock10::new();

type Bme680 = Bme680Controller<I2cBmeInstance, DelayWrapper<'static, ClockType>, StaticProvider>;

type NetworkStack = SslTcpStack<Esp8266IpNetworkDriver<'static, SerialTx>>;

#[exception]
fn MemoryManagement() -> ! {
    panic!("Memory management error")
}

#[exception]
fn HardFault(ef: &cortex_m_rt::ExceptionFrame) -> ! {
    // prints the exception frame as a panic message
    panic!("{:#?}", ef);
}

#[exception]
fn BusFault() -> ! {
    panic!("Bus fault");
}

#[exception]
fn UsageFault() -> ! {
    panic!("Usage fault");
}

#[cfg(feature = "stm32f411")]
fn create_serial(
    clocks: Clocks,
    instance: SerialInstance,
    rx_pin: <SerialInstance as SerialConfig<SerialInstance>>::RxPin,
    tx_pin: <SerialInstance as SerialConfig<SerialInstance>>::TxPin,
) -> SerialEsp {
    Serial::usart1(
        instance,
        (tx_pin, rx_pin),
        serial::config::Config {
            baudrate: 115_200.bps(),
            wordlength: serial::config::WordLength::DataBits8,
            parity: serial::config::Parity::ParityNone,
            stopbits: serial::config::StopBits::STOP1,
        },
        clocks,
    )
    .unwrap()
}

#[cfg(feature = "stm32f7xx")]
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

#[cfg(not(feature = "custom_wifi"))]
const WIFI_SSID: &'static str = "muenchen.freifunk.net/muc_ost";
#[cfg(not(feature = "custom_wifi"))]
const WIFI_PASSWORD: &'static str = "";

#[cfg(feature = "custom_wifi")]
const WIFI_SSID: &'static str = include_str!("wifi.ssid.txt");
#[cfg(feature = "custom_wifi")]
const WIFI_PASSWORD: &'static str = include_str!("wifi.password.txt");

/// Try joining the local Wi-Fi
#[cfg(feature = "publish")]
fn try_join<Tx>(adapter: &mut Adapter<'_, Tx>) -> Result<(), ThingError>
where
    Tx: embedded_hal::serial::Write<u8>,
{
    let result = adapter
        .get_firmware_info()
        .map_err(|_| ThingError::InitializationFailed)?;
    info!("firmware: {:?}", result);

    let result = adapter
        .set_mode(WiFiMode::Station)
        .map_err(|_| ThingError::InitializationFailed)?;
    info!("set mode {:?}", result);

    let result = adapter
        .join(WIFI_SSID, WIFI_PASSWORD)
        .map_err(|_| ThingError::InitializationFailed)?;
    info!("joined wifi {:?}", result);

    let result = adapter
        .get_ip_address()
        .map_err(|_| ThingError::InitializationFailed)?;
    info!("IP {:?}", result);

    let result = adapter
        .set_dns_resolvers(Ipv4Addr::new(8, 8, 8, 8), None)
        .map_err(|_| ThingError::InitializationFailed)?;
    info!("DNS: {:?}", result);

    Ok(())
}

/// Publish data to the cloud
#[cfg(feature = "publish")]
fn publish<T>(
    network: &mut T,
    data: Data,
    display: &mut Option<DisplayInstance>,
) -> Result<(), ThingError>
where
    T: TcpStack + Dns,
{
    if let Some(display) = display {
        display.set_state(State::Connecting);
    }

    log::info!("Resolve IP address");
    let addr = network
        .gethostbyname(ENDPOINT, AddrType::IPv4)
        .map_err(|_| ThingError::FailedToPublish)?;

    log::info!("Create socket");
    let socket = network
        .open(Mode::Blocking)
        .map_err(|_| ThingError::FailedToPublish)?;

    log::info!("Connect socket");
    let mut socket = network
        .connect(socket, HostSocketAddr::new(addr, ENDPOINT_PORT))
        .map_err(|_| ThingError::FailedToPublish)?;

    log::info!("do publish");
    let result = do_publish::<T>(network, &mut socket, data, display);

    info!("Closing socket");
    let r = network.close(socket);
    info!("Closing socket -> {:?}", r);

    result
}

#[cfg(feature = "publish")]
fn do_publish<T>(
    network: &mut T,
    socket: &mut T::TcpSocket,
    data: Data,
    display: &mut Option<DisplayInstance>,
) -> Result<(), ThingError>
where
    T: TcpStack,
{
    let mut tcp = TcpSocketSinkSource::from(network, socket);

    let con = HttpConnection::<consts::U1024>::new();

    let data: String<consts::U128> =
        serde_json_core::to_string(&data).map_err(|_| ThingError::FailedToPublish)?;

    let handler = BufferResponseHandler::<consts::U1024>::new();

    log::info!("Starting request...");

    if let Some(display) = display {
        display.set_state(State::SendingHeaders);
    }

    let mut req = con
        .post("/v1/telemetry")
        .headers(&[
            ("Host", ENDPOINT),
            ("Content-Type", "text/json"),
            (
                "Authorization",
                "Basic Y3Ryb24tc3RtMzJmNDExQGFwcDE6aGV5LXJvZG5leQ==",
            ),
        ])
        .handler(handler)
        .execute_with::<_, consts::U512>(&mut tcp, Some(data.as_ref()));

    if let Some(display) = display {
        display.set_state(State::SendingPayload);
    }

    log::info!("Request sent, piping data...");

    tcp.pipe_data(&mut req)
        .map_err(|_| ThingError::FailedToPublish)?;

    log::info!("Done piping data, checking result");

    let (_, handler) = req.complete();

    log::info!(
        "Result: {} {}, Payload: {:?}",
        handler.code(),
        handler.reason(),
        from_utf8(handler.payload())
    );

    if let Some(display) = display {
        display.set_result(handler.code() as i32);
    }

    Ok(())
}

#[cfg(feature = "publish")]
fn create_network(
    network: Esp8266IpNetworkDriver<SerialTx>,
) -> Result<SslTcpStack<Esp8266IpNetworkDriver<SerialTx>>, ThingError> {
    // create SSL
    let start = heap_start() as usize;
    info!("heap start: 0x{:0x?}", start);

    let mut ssl_platform =
        SslPlatform::setup(start, SSL_HEAP_SIZE).ok_or_else(|| ThingError::InitializationFailed)?;

    ssl_platform
        .entropy_context_mut()
        .add_source(StaticEntropySource);

    ssl_platform
        .seed_rng()
        .map_err(|_| ThingError::InitializationFailed)?;

    let mut ssl_config = ssl_platform
        .new_client_config(Transport::Stream, Preset::Default)
        .map_err(|_| ThingError::InitializationFailed)?;

    // FIXME: needs to be fixed, obviously
    ssl_config.authmode(Verify::None);

    // consume the config, take a non-mutable ref to the underlying network.
    Ok(SslTcpStack::new(ssl_config, network))
}

type DisplayInstance = crate::display::Display<
    Spi<
        SPI1,
        (
            PA5<Alternate<AF5>>,
            PA6<Alternate<AF5>>,
            PA7<Alternate<AF5>>,
        ),
    >,
    PC5<Output<PushPull>>,
    PC4<Output<PushPull>>,
>;

#[cfg(feature = "mpu")]
fn end_of_heap(size: Size) -> usize {
    let mut end = heap_start() as usize;
    end += SSL_HEAP_SIZE;

    let al = (1 << size.bits()) as usize;
    let diff = end % al;
    if diff != 0 {
        end += al - diff;
    }

    end
}

#[cfg(feature = "bsec")]
type Bsec = drogue_bsec::Bsec;
#[cfg(not(feature = "bsec"))]
type Bsec = ();

#[app(device = crate::device, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        #[init(None)]
        network: Option<NetworkStack>,
        adapter: Option<ESPAdapter>,
        ingress: Ingress<'static, SerialRx>,

        ticker: MillisecondsTicker10<'static, ClockType, Timer, fn(&mut Timer)>,

        sensor_i2c: Option<I2cBmeInstance>,
        #[init(None)]
        sensor: Option<Bme680>,
        display: Option<DisplayInstance>,
        #[init(0)]
        updates: u32,
        bsec: Bsec,
    }

    #[init(spawn = [digest, join, init_sensor])]
    fn init(ctx: init::Context) -> init::LateResources {
        //rtt_init_print!(NoBlockSkip, 1024);
        rtt_init_print!(BlockIfFull, 1024);
        log::set_logger(&LOGGER).unwrap();
        log::set_max_level(log::LevelFilter::Trace);

        let mut core = ctx.core;
        let device = ctx.device;

        // enable MPU
        #[cfg(all(feature = "stm32f4xx", feature = "mpu"))]
        {
            let mut regions = ArrayVec::new();
            let size = Size::S32B;
            let base = end_of_heap(size);
            regions.push(Region {
                base_addr: base,
                size,
                subregions: Subregions::ALL,
                executable: false,
                permissions: AccessPermission::NoAccess,
                attributes: MemoryAttributes::Normal {
                    shareable: false,
                    cache_policy: CachePolicy::NonCacheable,
                },
            });
            log::info!("Configure MPU - base: {:04x}, {:?}", base, size);
            let mut mpu = unsafe { Mpu::new(core.MPU) };
            mpu.configure_unprivileged(&regions);

            /*
            unsafe {
                let ptr = base as *mut u8;
                *ptr = 0;
            }
            */
        }

        // Enable CYCNT
        // Initialize (enable) the monotonic timer (CYCCNT)
        core.DCB.enable_trace();
        // required on Cortex-M7 devices that software lock the DWT (e.g. STM32F7)
        #[cfg(feature = "stm32f7xx")]
        DWT::unlock();
        core.DWT.enable_cycle_counter();

        #[cfg(feature = "stm32f4xx")]
        let rcc = device.RCC.constrain();
        #[cfg(feature = "stm32f7xx")]
        let mut rcc = device.RCC.constrain();

        #[cfg(feature = "stm32f4xx")]
        let clocks = rcc.cfgr.sysclk(100.mhz()).freeze();
        #[cfg(feature = "stm32f7xx")]
        let clocks = rcc.cfgr.sysclk(216.mhz()).freeze();

        let gpioa = device.GPIOA.split();
        let gpiob = device.GPIOB.split();
        let gpioc = device.GPIOC.split();
        #[cfg(feature = "stm32f723")]
        let gpiod = device.GPIOD.split();
        #[cfg(feature = "stm32f7xx")]
        let gpiog = device.GPIOG.split();
        #[cfg(feature = "stm32f723")]
        let gpioh = device.GPIOH.split();

        #[cfg(all(feature = "stm32f4xx", feature = "wait"))]
        {
            log::info!("Press user button to continue ...");
            let button = gpioc.pc13.into_pull_down_input();
            while button.is_high().unwrap() {
                cortex_m::asm::nop();
            }
        }

        #[cfg(feature = "stm32f4xx")]
        let (mut en, mut reset, mut esp_gpio0, mut esp_gpio2) = {
            let en = gpioc.pc0.into_push_pull_output();
            let reset = gpioc.pc1.into_push_pull_output();
            let esp_gpio0 = gpiob.pb0.into_push_pull_output();
            let esp_gpio2 = gpioa.pa4.into_push_pull_output();
            (en, reset, esp_gpio0, esp_gpio2)
        };
        #[cfg(feature = "stm32f7xx")]
        let (mut en, mut reset, mut esp_gpio0, mut esp_gpio2) = {
            let en = gpiod.pd3.into_push_pull_output();
            let reset = gpiog.pg14.into_push_pull_output();
            let esp_gpio0 = gpiog.pg13.into_push_pull_output();
            let esp_gpio2 = gpiod.pd6.into_push_pull_output();

            (en, reset, esp_gpio0, esp_gpio2)
        };

        // set both to HIGH (boot from flash, non-programming mode)
        esp_gpio0.set_high().unwrap();
        esp_gpio2.set_high().unwrap();

        #[cfg(feature = "stm32f411")]
        let mut serial = {
            let tx_pin = gpioa.pa9.into_alternate_af7();
            let rx_pin = gpioa.pa10.into_alternate_af7();

            create_serial(clocks, device.USART1, rx_pin, tx_pin)
        };
        #[cfg(feature = "stm32f723")]
        let mut serial = {
            let tx_pin = gpioc.pc12.into_alternate_af8();
            let rx_pin = gpiod.pd2.into_alternate_af8();

            create_serial(clocks, device.UART5, rx_pin, tx_pin)
        };

        serial.listen(serial::Event::Rxne);
        let (tx, rx) = serial.split();

        info!("Serial buffer size: {}", BUFFER_LEN);

        static mut RESPONSE_QUEUE: Queue<Response, consts::U2> = Queue(i::Queue::new());
        static mut NOTIFICATION_QUEUE: Queue<Response, consts::U16> = Queue(i::Queue::new());

        info!("Init ESP");

        let (adapter, ingress) = initialize(
            tx,
            rx,
            &mut en,
            &mut reset,
            unsafe { &mut RESPONSE_QUEUE },
            unsafe { &mut NOTIFICATION_QUEUE },
        )
        .unwrap();

        // init timer

        info!("Init timer");

        #[cfg(feature = "stm32f411")]
        let mut hal_hz_timer = hal::timer::Timer::tim3(device.TIM3, 100.hz(), clocks);
        #[cfg(feature = "stm32f723")]
        let mut hal_hz_timer =
            hal::timer::Timer::tim2(device.TIM2, 100.hz(), clocks, &mut rcc.apb1);

        hal_hz_timer.listen(Event::TimeOut);
        let mut ticker = CLOCK.ticker(
            hal_hz_timer,
            (|t| {
                #[cfg(feature = "stm32f4xx")]
                t.clear_interrupt(Event::TimeOut);
            }) as fn(&mut Timer),
        );
        ticker.tick();

        // init i2c

        info!("Init I2C");

        #[cfg(feature = "stm32f4xx")]
        let (sda, scl) = {
            let sda = gpiob.pb9.into_alternate_af4_open_drain();
            let scl = gpiob.pb8.into_alternate_af4_open_drain();
            (sda, scl)
        };
        #[cfg(feature = "stm32f7xx")]
        let (sda, scl) = {
            let sda = gpioh.ph5.into_alternate_af4();
            let scl = gpioh.ph4.into_alternate_af4();
            (sda, scl)
        };

        // Initialize I2C

        #[cfg(feature = "stm32f411")]
        let i2c = I2c::i2c1(device.I2C1, (scl, sda), 100.khz(), clocks);
        #[cfg(feature = "stm32f723")]
        let i2c = BlockingI2c::i2c2(
            device.I2C2,
            (scl, sda),
            i2c::Mode::standard(100_000.hz()),
            clocks,
            &mut rcc.apb1,
            1000,
        );

        #[cfg(feature = "display")]
        let display = if cfg!(feature = "display") {
            info!("Init display");

            // init display
            let mosi = gpioa.pa7.into_alternate_af5();
            let miso = gpioa.pa6.into_alternate_af5();
            let sck = gpioa.pa5.into_alternate_af5();

            let mut cs = gpioc.pc13.into_push_pull_output();
            let dc = gpioc.pc5.into_push_pull_output();
            let rst = gpioc.pc4.into_push_pull_output();

            cs.set_low().unwrap();

            let spi = Spi::spi1(
                device.SPI1,
                (sck, miso, mosi),
                MODE_0,
                1000.khz().into(),
                clocks,
            );
            Some(Display::new(spi, dc, rst))
        } else {
            None
        };

        #[cfg(feature = "bsec")]
        let bsec = {
            info!("BSEC version: {}", bsec::version());
            let mut result = Bsec::new().unwrap().unwrap();
            result
                .update_subscription(
                    SampleRate::LowPower,
                    &[
                        bsec::bsec_virtual_sensor_t::BSEC_OUTPUT_RAW_TEMPERATURE,
                        bsec::bsec_virtual_sensor_t::BSEC_OUTPUT_RAW_PRESSURE,
                        bsec::bsec_virtual_sensor_t::BSEC_OUTPUT_RAW_HUMIDITY,
                        bsec::bsec_virtual_sensor_t::BSEC_OUTPUT_RAW_GAS,
                        bsec::bsec_virtual_sensor_t::BSEC_OUTPUT_IAQ,
                        bsec::bsec_virtual_sensor_t::BSEC_OUTPUT_STATIC_IAQ,
                        bsec::bsec_virtual_sensor_t::BSEC_OUTPUT_CO2_EQUIVALENT,
                        bsec::bsec_virtual_sensor_t::BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
                        bsec::bsec_virtual_sensor_t::BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
                        bsec::bsec_virtual_sensor_t::BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
                    ],
                )
                .unwrap();
            info!("BSEC initialized");
            result
        };
        #[cfg(not(feature = "bsec"))]
        let bsec = {
            info!("BSEC version: <disabled>");
            ()
        };

        info!("Spawn tasks");

        // spawn others

        ctx.spawn.digest().unwrap();
        ctx.spawn.init_sensor().unwrap();
        ctx.spawn.join().unwrap();

        // done

        info!("initialized");

        init::LateResources {
            adapter: Some(adapter),
            ingress,
            ticker,
            sensor_i2c: Some(i2c),
            display,
            bsec,
        }
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::nop()
        }
    }

    #[task(schedule = [digest], priority = 3, resources = [ingress])]
    fn digest(mut ctx: digest::Context) {
        ctx.resources.ingress.lock(|ingress| ingress.digest());
        ctx.schedule
            .digest(ctx.scheduled + (DIGEST_DELAY * 100_000).cycles())
            .unwrap();
    }

    // comment out, due to: https://github.com/rtic-rs/cortex-m-rtic/issues/387
    #[cfg(feature = "stm32f411")]
    #[task(binds = USART1, priority = 20, resources = [ingress])]
    fn usart(ctx: usart::Context) {
        if let Err(b) = ctx.resources.ingress.isr() {
            info!("failed to ingress {}", b as char);
        }
    }

    // comment out, due to: https://github.com/rtic-rs/cortex-m-rtic/issues/387
    /*
       #[cfg(feature = "stm32f723")]
       #[task(binds = UART5, priority = 20, resources = [ingress])]
       fn usart(ctx: usart::Context) {
           if let Err(b) = ctx.resources.ingress.isr() {
               info!("failed to ingress {}", b as char);
           }
       }
    */

    #[task(resources = [sensor, sensor_i2c, display])]
    fn init_sensor(ctx: init_sensor::Context) {
        #[cfg(feature = "display")]
        {
            log::info!("Init display...");
            let mut delay = CLOCK.delay();
            ctx.resources
                .display
                .as_mut()
                .unwrap()
                .init(&mut delay)
                .unwrap();
        }

        log::info!("Init sensor...");

        let i2c = ctx.resources.sensor_i2c.take().unwrap();

        let bme680 = Bme680Sensor::from(i2c, Address::Secondary).unwrap();

        log::info!("Sensor initialized ... initializing controller ...");

        let controller = Bme680Controller::new(
            bme680,
            DelayWrapper(CLOCK.delay()),
            Configuration::standard(),
            StaticProvider(25),
        )
        .unwrap();

        log::info!("Controller initialized!");

        if let Some(display) = ctx.resources.display {
            display.set_state(State::Initialized);
        }

        ctx.resources.sensor.replace(controller);
    }

    #[task(schedule = [join], spawn = [update], resources = [adapter, network, display])]
    fn join(ctx: join::Context) {
        #[cfg(feature = "publish")]
        {
            log::info!("Joining network");

            if ctx.resources.network.is_some() {
                return;
            }

            if ctx.resources.adapter.is_none() {
                panic!("No adapter and no network");
            }

            if let Some(display) = ctx.resources.display {
                display.set_state(State::Joining);
            }

            let mut adapter = ctx.resources.adapter.take().unwrap();

            match try_join(&mut adapter) {
                Ok(_) => {
                    // we joined the local Wi-Fi
                    info!("network initialized");
                    // convert the adapter into a TCP stack, this consumes the adapter instance
                    let network = adapter.into_network_stack();
                    // and wrap it with a TLS layer
                    let ssl = create_network(network).unwrap();
                    // store that in our context
                    ctx.resources.network.replace(ssl);

                    if let Some(display) = ctx.resources.display {
                        display.set_state(State::Joined);
                    }

                    // joined ... update
                    ctx.spawn.update().unwrap();
                }
                Err(_) => {
                    warn!("Failed to join Wi-Fi, trying again later");
                    // we failed to join the local Wi-Fi, put back the adapter to try again later
                    ctx.resources.adapter.replace(adapter);
                    // try joining later
                    ctx.schedule
                        .join(ctx.scheduled + 8_000_000.cycles())
                        .unwrap();
                }
            }
        }
        #[cfg(not(feature = "publish"))]
        {
            ctx.spawn.update().unwrap();
        }
    }

    #[task(spawn = [], schedule=[update], resources = [network, sensor, display, updates, bsec])]
    fn update(ctx: update::Context) {
        log::info!("Updating");

        let network: &mut Option<_> = ctx.resources.network;
        let sensor: &mut Option<Bme680> = ctx.resources.sensor;
        let display: &mut Option<DisplayInstance> = ctx.resources.display;

        let bsec: &mut Bsec = ctx.resources.bsec;

        *ctx.resources.updates += 1;

        if let Some(display) = display {
            display.set_iter(*ctx.resources.updates);
        }

        log::info!("Start update");

        perform_publish(
            #[cfg(feature = "publish")]
            network,
            sensor,
            display,
            bsec,
        )
        .ok();

        // done publishing ... successful or not

        log::info!("Done updating ... schedule again");

        // ctx.spawn.next_update().unwrap();
        ctx.schedule
            .update(ctx.scheduled + 1_500_000_000u32.cycles())
            .unwrap();
    }

    // workaround, to scheduled x cycles after the publish job as finished, not x cycles after
    // the publish job was started
    #[task(schedule = [update])]
    fn next_update(ctx: next_update::Context) {
        log::info!("Schedule next update");
        let cycles = (1_500_000_000u32).cycles();
        //ctx.schedule.update(ctx.scheduled + cycles).unwrap();
    }

    #[cfg(any(feature = "stm32f4xx", feature = "stm32f7xx"))]
    #[task(binds = TIM3, priority = 15, resources = [ticker])]
    fn ticker(ctx: ticker::Context) {
        ctx.resources.ticker.tick();
    }

    /*
    #[task(binds = TIM2, priority = 15, resources = [ticker])]
    fn ticker(ctx: ticker::Context) {
        ctx.resources.ticker.tick();
    }
    */

    // spare interrupt used for scheduling software tasks
    extern "C" {
        fn SPI1();
        fn SPI2();
        fn SPI3();
        fn SPI4();
        fn SPI5();
    }
};

fn now() -> Milliseconds {
    let now = CLOCK.try_now().unwrap();
    let now = now.duration_since_epoch();
    now.try_into().unwrap()
}

#[cfg(not(feature = "bsec"))]
fn perform_publish(
    #[cfg(feature = "publish")] network: &mut Option<NetworkStack>,
    sensor: &mut Option<Bme680>,
    display: &mut Option<DisplayInstance>,
    bsec: &mut Bsec,
) -> Result<(), ThingError> {
    let mut sensor = sensor.as_mut().ok_or_else(|| ThingError::NotInitialized)?;

    if let Some(display) = display {
        display.set_state(State::Measuring).ok();
    }

    let data = data::measure(sensor);
    log::info!("Measured: {:?}", data);
    let data = data.map_err(|_| ThingError::FailedToMeasure)?;

    if let Some(display) = display {
        display.set_temp(data.temperature).ok();
    }

    #[cfg(feature = "publish")]
    {
        let network = network.as_mut().ok_or_else(|| ThingError::NotInitialized)?;

        // publish measurement
        publish(network, data, display)?;
    }

    log::info!("Setting state");

    if let Some(display) = display {
        display.set_state(State::Done).ok();
    }

    log::info!("Done");

    Ok(())
}

#[cfg(feature = "bsec")]
fn perform_publish(
    #[cfg(feature = "publish")] network: &mut Option<NetworkStack>,
    sensor: &mut Option<Bme680>,
    display: &mut Option<DisplayInstance>,
    bsec: &mut Bsec,
) -> Result<(), ThingError> {
    let mut sensor = sensor.as_mut().ok_or_else(|| ThingError::NotInitialized)?;

    if let Some(display) = display {
        display.set_state(State::Measuring).ok();
    }

    loop {
        let ts = now();
        log::info!("Timestamp: {}", ts);
        let result = bsec.sensor_control(ts);
        log::info!("BSEC control: {:?}", &result);
        if let Ok(control) = result {
            log::info!("next: {}, ts: {}", control.next_call, ts);
            if control.next_call > ts {
                break;
            }
            bsec.process_data(ts, &Inputs::default());
        }
        // CLOCK.delay().delay(Milliseconds(10u32));
    }
    //let result = result.map_err(|_| ThingError::FailedToMeasure)?;

    // measure data
    let data = data::measure(&mut sensor)?;
    log::info!("Measured: {:?}", data);

    let ts = now();
    log::info!("Timestamp: {}", ts);

    let result = bsec.process_data(
        ts,
        &Inputs {
            temperature: Some(data.temperature),
            humidity: Some(data.humidity),
            pressure: data.pressure,
            gas_resistance: Some(data.gas_resistance),
        },
    );

    log::info!("Back from BSEC: {:?}", result);

    if let Some(display) = display {
        display.set_temp(data.temperature).ok();

        #[cfg(feature = "bsec")]
        match result {
            Ok(r) => display.set_details(&r),
            Err(e) => display.set_details(&Outputs::default()),
        };
    }

    #[cfg(feature = "publish")]
    {
        let network = network.as_mut().ok_or_else(|| ThingError::NotInitialized)?;

        // publish measurement
        publish(network, data, display)?;
    }

    log::info!("Setting state");

    if let Some(display) = display {
        display.set_state(State::Done).ok();
    }

    log::info!("Done");

    Ok(())
}
