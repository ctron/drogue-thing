use core::fmt::Write;
#[cfg(feature = "bsec")]
use drogue_bsec::{Accuracy, Outputs};
use embedded_graphics::drawable::Drawable;
use embedded_graphics::fonts::{Font, Font24x32, Font6x12, Font6x8, Font8x16, Text};
use embedded_graphics::geometry::Point;
use embedded_graphics::pixelcolor::{Rgb565, RgbColor};
use embedded_graphics::primitives::{Primitive, Rectangle};
use embedded_graphics::style::{PrimitiveStyle, Styled, TextStyle};
use embedded_graphics::DrawTarget;
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::digital::v2::OutputPin;
use heapless::consts;
use heapless::String;
use ssd1351::builder::Builder;
use ssd1351::interface::SpiInterface;
use ssd1351::mode::GraphicsMode;

#[derive(Copy, Clone, Debug)]
pub enum State {
    Start,
    Initialized,
    Joining,
    Joined,
    Measuring,
    Connecting,
    SendingHeaders,
    SendingPayload,
    Done,
}

pub struct Display<SPI, DC, RST>
where
    SPI: embedded_hal::blocking::spi::Transfer<u8> + embedded_hal::blocking::spi::Write<u8>,
    DC: OutputPin,
    RST: OutputPin + Sized,
{
    inner: GraphicsMode<SpiInterface<SPI, DC>>,
    rst: RST,
}

impl<SPI, DC, RST> Display<SPI, DC, RST>
where
    SPI: embedded_hal::blocking::spi::Transfer<u8> + embedded_hal::blocking::spi::Write<u8>,
    DC: OutputPin,
    RST: OutputPin + Sized,
{
    pub fn new(spi: SPI, dc: DC, rst: RST) -> Self {
        let display: GraphicsMode<_> = Builder::new().connect_spi(spi, dc).into();
        Display {
            inner: display,
            rst,
        }
    }

    pub fn init<D>(&mut self, delay: &mut D) -> Result<(), ()>
    where
        D: DelayMs<u8>,
    {
        self.inner.reset(&mut self.rst, delay).map_err(|_| ())?;
        self.inner.init()?;
        Ok(())
    }

    pub fn set_text(&mut self, text: &str, top: i32, left: i32, width: i32) -> Result<(), ()> {
        self.clear_rect(Rectangle::new(Point::new(left, top), Point::new(width, 20)))?;

        Text::new(text, Point::new(left, top))
            .into_styled(TextStyle::new(Font8x16, Rgb565::new(255, 255, 255)))
            .draw(&mut self.inner)
    }

    pub fn set_temp(&mut self, temp: f32) -> Result<(), ()> {
        let mut buffer: String<consts::U16> = String::new();
        write!(buffer, "{:.2} °C", temp).map_err(|_| ())?;

        let width = 96;
        self.clear_rect(Rectangle::new(Point::new(0, 0), Point::new(width, 16)))?;

        Text::new(&buffer, Point::zero())
            .into_styled(TextStyle::new(Font8x16, Rgb565::new(255, 255, 255)))
            .draw(&mut self.inner)
    }

    pub fn set_result(&mut self, code: i32) -> Result<(), ()> {
        let mut buffer: String<consts::U16> = String::new();
        write!(buffer, "Code: {}", code).map_err(|_| ())?;

        let top = 20;
        let width = 96;
        self.clear_rect(Rectangle::new(
            Point::new(0, top),
            Point::new(width, top + 16),
        ))?;

        Text::new(&buffer, Point::new(0, top))
            .into_styled(TextStyle::new(Font8x16, Rgb565::new(255, 255, 255)))
            .draw(&mut self.inner)
    }

    pub fn set_iter(&mut self, i: u32) -> Result<(), ()> {
        let mut buffer: String<consts::U16> = String::new();
        write!(buffer, "{:>3}", i).map_err(|_| ())?;

        let left = 96;
        let size = self.inner.size();
        self.clear_rect(Rectangle::new(
            Point::new(left, 0),
            Point::new(size.width as i32, 32),
        ))?;

        Text::new(&buffer, Point::new(left, 8))
            .into_styled(TextStyle::new(Font8x16, Rgb565::new(255, 255, 255)))
            .draw(&mut self.inner)
    }

    pub fn set_state(&mut self, state: State) -> Result<(), ()> {
        let mut buffer: String<consts::U16> = String::new();

        write!(buffer, "{:?}", state).map_err(|_| ())?;

        let height = 14;
        let size = self.inner.size();
        let top = self.inner.size().height as i32 - height;

        self.clear_rect(Rectangle::new(
            Point::new(0, top),
            Point::new(size.width as i32, size.height as i32),
        ))?;

        Text::new(&buffer, Point::new(0, top))
            .into_styled(TextStyle::new(Font6x12, Rgb565::new(255, 255, 255)))
            .draw(&mut self.inner)
    }

    #[cfg(feature = "bsec")]
    fn output_as_text<F>(
        &mut self,
        label: &str,
        value: &Option<Accuracy<f32>>,
        font: F,
        top: i32,
        left: i32,
        width: i32,
    ) -> Result<(), ()>
    where
        F: Font + Copy,
    {
        let mut buffer: String<consts::U16> = String::new();

        let height = F::CHARACTER_SIZE.height as i32 + 2;

        match value {
            Some(Accuracy::Low(v)) | Some(Accuracy::Medium(v)) | Some(Accuracy::High(v)) => {
                write!(buffer, "{}: {}", label, v)
            }
            _ => write!(buffer, "{}: <??>", label),
        }
        .map_err(|_| ())?;

        self.clear_rect(Rectangle::new(
            Point::new(left, top),
            Point::new(left + width, top + height),
        ))?;

        let color = match value {
            None => Rgb565::CYAN,
            Some(Accuracy::Unreliable) => Rgb565::MAGENTA,
            Some(Accuracy::Low(_)) => Rgb565::RED,
            Some(Accuracy::Medium(_)) => Rgb565::YELLOW,
            Some(Accuracy::High(_)) => Rgb565::GREEN,
        };

        Text::new(&buffer, Point::new(left, top + 1))
            .into_styled(TextStyle::new(font, color))
            .draw(&mut self.inner)?;

        Ok(())
    }

    #[cfg(feature = "bsec")]
    pub fn set_details(&mut self, outputs: &Outputs) -> Result<(), ()> {
        let size = self.inner.size();

        let top = 40;
        let height = 10;
        let half = size.width as i32 / 2;
        self.output_as_text("IAQ", &outputs.iaq, Font6x8, top, 0, half)?;
        self.output_as_text("GAS", &outputs.gas_percentage, Font6x8, top, half, half)?;

        self.output_as_text(
            "CO2",
            &outputs.co2_equivalent,
            Font6x8,
            top + height,
            0,
            half,
        )?;
        self.output_as_text(
            "VOC",
            &outputs.breath_voc_equivalent,
            Font6x8,
            top + height,
            half,
            half,
        )?;

        Ok(())
    }

    fn clear_rect(&mut self, rect: Rectangle) -> Result<(), ()> {
        rect.into_styled(PrimitiveStyle::with_fill(Rgb565::BLACK))
            .draw(&mut self.inner)
    }
}
