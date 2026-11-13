//! Host stand-ins for the hardware the pre-refactor `system.rs` names. They
//! replace hardware only: the PWM keeps its trim arithmetic (copied from
//! `elle_hardware::pwm`), the events and writes are recorded, and the watchdog
//! does nothing. No control law lives here.

use std::cell::RefCell;

/// What one tick did to the outside world, in order.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Effect {
    /// `set_elevons_with_trim(left, right)` → pulses actually output.
    Trimmed {
        left_us: u32,
        right_us: u32,
        pulses: (u32, u32),
    },
    /// `set_safe_positions()`.
    Safe,
    /// `elle_event!(level, code, …)`, with its message formatted.
    Event { level: u8, code: u16, msg: String },
}

thread_local! {
    pub static EFFECTS: RefCell<Vec<Effect>> = const { RefCell::new(Vec::new()) };
}

pub fn record(effect: Effect) {
    EFFECTS.with(|e| e.borrow_mut().push(effect));
}

pub fn take_effects() -> Vec<Effect> {
    EFFECTS.with(|e| core::mem::take(&mut *e.borrow_mut()))
}

pub mod elle_hardware {
    pub mod event {
        // Same values as `elle_hardware::event`.
        pub const EVT_RC_WARNING: u16 = 13;
        pub const EVT_RC_SIGNAL_LOST: u16 = 14;
        pub const EVT_RC_RESTORED: u16 = 15;
        pub const EVT_CORE1_UNHEALTHY: u16 = 70;
        pub const EVT_CORE1_RESTORED: u16 = 71;
        pub const EVT_ATTITUDE_STALE: u16 = 80;
        pub const EVT_ATTITUDE_HELD: u16 = 83;

        pub const fn level(name: &str) -> u8 {
            match name.as_bytes() {
                b"trace" => 0,
                b"debug" => 1,
                b"info" => 2,
                b"warn" => 3,
                _ => 4,
            }
        }
    }

    macro_rules! elle_event {
        ($level:ident, $code:expr, $($arg:tt)*) => {{
            $crate::shims::record($crate::shims::Effect::Event {
                level: $crate::shims::elle_hardware::event::level(stringify!($level)),
                code: $code,
                msg: format!($($arg)*),
            });
        }};
    }
    pub(crate) use elle_event;

    pub mod imu {
        use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
        use embassy_sync::signal::Signal;
        use embassy_time::Instant;

        pub use elle_control::flight_law::Attitude as AttitudeData;

        pub static CORE1_HEARTBEAT: Signal<CriticalSectionRawMutex, ()> = Signal::new();

        // Verbatim from `elle_hardware::imu`.
        pub fn is_attitude_valid(attitude: &AttitudeData, max_age: embassy_time::Duration) -> bool {
            attitude.timestamp != Instant::from_ticks(0) && attitude.timestamp.elapsed() < max_age
        }
    }

    pub mod pwm {
        use core::marker::PhantomData;
        use elle_config::*;
        use embassy_time::Instant;

        pub struct PwmOutputs<'a>(PhantomData<&'a ()>);

        impl PwmOutputs<'_> {
            pub const fn new() -> Self {
                Self(PhantomData)
            }

            pub fn set_safe_positions(&mut self) -> u16 {
                super::super::record(super::super::Effect::Safe);
                0
            }

            // Trim and inversion verbatim from `elle_hardware::pwm`.
            pub fn set_elevons_with_trim(
                &mut self,
                left_us: u32,
                right_us: u32,
            ) -> (u32, u32, u16) {
                let left = apply_elevon_trim(left_us, ELEVON_LEFT_TRIM_US);
                let inverted_right_us = SERVO_MAX_PULSE_US + SERVO_MIN_PULSE_US - right_us;
                let right = apply_elevon_trim(inverted_right_us, ELEVON_RIGHT_TRIM_US);
                super::super::record(super::super::Effect::Trimmed {
                    left_us,
                    right_us,
                    pulses: (left, right),
                });
                (left, right, 0)
            }

            pub fn next_wrap(&self) -> Instant {
                Instant::now()
            }
        }

        impl elle_control::flight_law::ElevonOutput for PwmOutputs<'_> {
            fn set_elevons_with_trim(&mut self, left_us: u32, right_us: u32) -> (u32, u32, u16) {
                PwmOutputs::set_elevons_with_trim(self, left_us, right_us)
            }

            fn set_safe_positions(&mut self) -> u16 {
                PwmOutputs::set_safe_positions(self)
            }
        }

        fn apply_elevon_trim(base_us: u32, trim_us: i32) -> u32 {
            let clamped_trim = trim_us.clamp(-MAX_TRIM_US, MAX_TRIM_US);
            let trimmed = (base_us as i32 + clamped_trim) as u32;
            trimmed.clamp(SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US)
        }
    }

    pub mod watchdog {
        use embassy_time::Duration;
        pub fn install(
            _watchdog: super::super::embassy_rp::watchdog::Watchdog<'static>,
            _timeout: Duration,
        ) {
        }
        pub fn feed(_timeout: Duration) {}
    }
}

pub mod embassy_rp {
    pub mod watchdog {
        pub struct Watchdog<'a>(core::marker::PhantomData<&'a ()>);
    }
}

/// defmt needs a logger to link; nothing is printed.
#[defmt::global_logger]
struct NullLogger;

unsafe impl defmt::Logger for NullLogger {
    fn acquire() {}
    unsafe fn flush() {}
    unsafe fn release() {}
    unsafe fn write(_bytes: &[u8]) {}
}

#[defmt::panic_handler]
fn defmt_panic() -> ! {
    panic!("defmt panic")
}
