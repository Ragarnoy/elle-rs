use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use elle_rpc_icd::LogMsg;

pub static LOG_CHANNEL: Channel<CriticalSectionRawMutex, LogMsg, 16> = Channel::new();

pub fn send(level: u8, code: u16) {
    let _ = LOG_CHANNEL.try_send(LogMsg { level, code });
}
