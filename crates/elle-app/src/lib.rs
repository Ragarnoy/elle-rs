//! The Elle flight application, shared by every airframe binary.
//!
//! Everything here is platform-independent: the flight and RPC control loops,
//! RPC dispatch, the boot sequence, ULog recording and the shared tasks.
//! Platform differences come from `elle-config` (`platform-dart`) and
//! `elle-hardware` (`single-engine`); each binary keeps only pins, engine setup,
//! interrupt bindings and `main()`.
#![no_std]

#[cfg(all(feature = "rpc-rc", not(feature = "rpc-control")))]
compile_error!("rpc-rc requires rpc-control (RC command source only applies in RPC mode)");

pub mod engines;
pub mod logging;
pub mod support;

#[cfg(feature = "rpc-control")]
pub mod flight_state;
#[cfg(feature = "rpc-control")]
pub mod rpc_app;
#[cfg(feature = "rpc-control")]
pub mod rpc_handlers;

/// Latest raw RC channels, for the `GetRcChannels` RPC handler.
#[cfg(feature = "rpc-control")]
pub mod rc_signal {
    use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
    use embassy_sync::signal::Signal;

    pub static RC_SIGNAL: Signal<CriticalSectionRawMutex, [u16; 16]> = Signal::new();
}
