//! Host side of the Elle flight controller's RPC link: the probe and RTT
//! bridge, the postcard-RPC client, and event labels. Shared by the TUI, the
//! `direct` commands and the MCP server.

pub mod events;
pub mod link;
pub mod mcp;
pub mod probe;
pub mod wire;
