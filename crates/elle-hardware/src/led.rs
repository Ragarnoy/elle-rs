//! WS2812B RGB LED driver for status indication

use embassy_rp::Peri;
use embassy_rp::peripherals::DMA_CH2;
use embassy_rp::pio::{Common, StateMachine};
use embassy_rp::pio_programs::ws2812::{PioWs2812, PioWs2812Program};
use embassy_time::{Duration, Timer};
use smart_leds::RGB8;

/// LED colors for different states
pub mod colors {
    use smart_leds::RGB8;

    pub const OFF: RGB8 = RGB8 { r: 0, g: 0, b: 0 };
    pub const RED: RGB8 = RGB8 { r: 50, g: 0, b: 0 };
    pub const GREEN: RGB8 = RGB8 { r: 0, g: 50, b: 0 };
    pub const BLUE: RGB8 = RGB8 { r: 0, g: 0, b: 50 };
    pub const YELLOW: RGB8 = RGB8 { r: 50, g: 50, b: 0 };
    pub const ORANGE: RGB8 = RGB8 { r: 50, g: 20, b: 0 };
    pub const PURPLE: RGB8 = RGB8 { r: 50, g: 0, b: 50 };
    pub const CYAN: RGB8 = RGB8 { r: 0, g: 50, b: 50 };
    pub const WHITE: RGB8 = RGB8 {
        r: 30,
        g: 30,
        b: 30,
    };
}

/// LED patterns for different system states
#[derive(Clone, Copy, Debug)]
pub enum LedPattern {
    Off,
    Solid(RGB8),
    SlowBlink(RGB8),   // 1Hz - Initializing
    FastBlink(RGB8),   // 4Hz - Calibrating
    DoubleBlink(RGB8), // Double flash - Normal operation
    RapidFlash(RGB8),  // 10Hz - Error
    Pulse(RGB8),       // Breathing effect
    Rainbow,           // Rainbow cycle for special modes
}

pub struct StatusLed<'a, PIO, const SM: usize>
where
    PIO: embassy_rp::pio::Instance,
{
    ws2812: PioWs2812<'a, PIO, SM, 1>,
    current_pattern: LedPattern,
    pattern_counter: u32,
    brightness: u8,
}

impl<PIO, const SM: usize> StatusLed<'_, PIO, SM>
where
    PIO: embassy_rp::pio::Instance,
{
    pub fn new(
        common: &mut Common<'static, PIO>,
        sm: StateMachine<'static, PIO, SM>,
        pin: Peri<'static, impl embassy_rp::pio::PioPin>,
        dma: Peri<'static, DMA_CH2>,
    ) -> Self {
        let program = PioWs2812Program::new(common);
        let ws2812 = PioWs2812::new(common, sm, dma, pin, &program);

        Self {
            ws2812,
            current_pattern: LedPattern::Off,
            pattern_counter: 0,
            brightness: 100, // Default brightness percentage
        }
    }

    /// Set the LED brightness (0-100%)
    pub fn set_brightness(&mut self, brightness: u8) {
        self.brightness = brightness.min(100);
    }

    /// Apply brightness scaling to a color
    fn scale_brightness(&self, color: RGB8) -> RGB8 {
        RGB8 {
            r: (color.r as u16 * self.brightness as u16 / 100) as u8,
            g: (color.g as u16 * self.brightness as u16 / 100) as u8,
            b: (color.b as u16 * self.brightness as u16 / 100) as u8,
        }
    }

    /// Set a new LED pattern
    pub async fn set_pattern(&mut self, pattern: LedPattern) {
        self.current_pattern = pattern;
        self.pattern_counter = 0;
        self.update().await;
    }

    /// Update the LED based on current pattern
    pub async fn update(&mut self) {
        let color = match self.current_pattern {
            LedPattern::Off => colors::OFF,

            LedPattern::Solid(color) => self.scale_brightness(color),

            LedPattern::SlowBlink(color) => {
                // 1Hz blink
                if (self.pattern_counter / 50).is_multiple_of(2) {
                    self.scale_brightness(color)
                } else {
                    colors::OFF
                }
            }

            LedPattern::FastBlink(color) => {
                // 4Hz blink
                if (self.pattern_counter / 12).is_multiple_of(2) {
                    self.scale_brightness(color)
                } else {
                    colors::OFF
                }
            }

            LedPattern::DoubleBlink(color) => {
                // Double flash every second
                let phase = self.pattern_counter % 100;
                if phase < 5 || (10..15).contains(&phase) {
                    self.scale_brightness(color)
                } else {
                    colors::OFF
                }
            }

            LedPattern::RapidFlash(color) => {
                // 10Hz flash
                if (self.pattern_counter / 5).is_multiple_of(2) {
                    self.scale_brightness(color)
                } else {
                    colors::OFF
                }
            }

            LedPattern::Pulse(color) => {
                // Breathing effect
                let phase = (self.pattern_counter % 100) as f32;
                let brightness = if phase < 50.0 {
                    phase / 50.0
                } else {
                    2.0 - (phase / 50.0)
                };

                RGB8 {
                    r: (color.r as f32 * brightness * self.brightness as f32 / 100.0) as u8,
                    g: (color.g as f32 * brightness * self.brightness as f32 / 100.0) as u8,
                    b: (color.b as f32 * brightness * self.brightness as f32 / 100.0) as u8,
                }
            }

            LedPattern::Rainbow => {
                // Rainbow cycle
                wheel((self.pattern_counter * 2) as u8)
            }
        };

        let data = [color; 1];
        self.ws2812.write(&data).await;

        self.pattern_counter = self.pattern_counter.wrapping_add(1);
    }

    /// Run the LED update loop
    pub async fn run(&mut self) {
        loop {
            self.update().await;
            Timer::after(Duration::from_millis(10)).await;
        }
    }

    /// Quick helper to show a color briefly
    pub async fn flash(&mut self, color: RGB8, duration_ms: u64) {
        let scaled = self.scale_brightness(color);
        self.ws2812.write(&[scaled; 1]).await;
        Timer::after(Duration::from_millis(duration_ms)).await;
        self.ws2812.write(&[colors::OFF; 1]).await;
    }

    /// Show calibration progress with color gradient
    pub async fn show_calibration_progress(&mut self, sys: u8, gyro: u8, accel: u8, mag: u8) {
        // Calculate overall progress (0-12 total)
        let total = sys + gyro + accel + mag;

        let color = if total < 4 {
            colors::RED // Poor calibration
        } else if total < 8 {
            colors::ORANGE // Getting there
        } else if total < 11 {
            colors::YELLOW // Almost ready
        } else {
            colors::GREEN // Fully calibrated
        };

        self.set_pattern(LedPattern::Pulse(color)).await;
    }
}

/// Helper function to generate rainbow colors
fn wheel(mut wheel_pos: u8) -> RGB8 {
    wheel_pos = 255 - wheel_pos;
    if wheel_pos < 85 {
        return RGB8 {
            r: (255 - wheel_pos * 3) / 5, // Scaled down for brightness
            g: 0,
            b: (wheel_pos * 3) / 5,
        };
    }
    if wheel_pos < 170 {
        wheel_pos -= 85;
        return RGB8 {
            r: 0,
            g: (wheel_pos * 3) / 5,
            b: (255 - wheel_pos * 3) / 5,
        };
    }
    wheel_pos -= 170;
    RGB8 {
        r: (wheel_pos * 3) / 5,
        g: (255 - wheel_pos * 3) / 5,
        b: 0,
    }
}
