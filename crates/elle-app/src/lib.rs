//! The Elle flight application, shared by every airframe binary.
//!
//! Everything here is platform-independent: the flight and RPC control loops,
//! RPC dispatch, the boot sequence, ULog recording and the shared tasks.
//! Platform differences come from `elle-config` (`platform-dart`) and
//! `elle-hardware` (`single-engine`); each binary keeps only pins, engine setup,
//! interrupt bindings and `main()`.
#![no_std]
#![allow(clippy::too_many_arguments)] // embassy task macros generate wrapper fns

#[cfg(all(feature = "rpc-rc", not(feature = "rpc-control")))]
compile_error!("rpc-rc requires rpc-control (RC command source only applies in RPC mode)");

pub mod boot;
pub mod engines;
pub mod logging;
pub mod support;
pub mod tasks;

#[cfg(not(feature = "rpc-control"))]
pub mod flight;
#[cfg(feature = "rpc-control")]
pub mod rpc;

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
