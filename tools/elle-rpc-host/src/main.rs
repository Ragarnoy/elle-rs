//! Elle RPC Host CLI
//!
//! Communicate with the Elle flight controller via postcard-RPC.
//!
//! Modes:
//! - TCP (default): Connects to cargo-embed via TCP sockets
//! - Direct: Uses probe-rs directly (requires exclusive probe access)

mod commands;
mod direct;
mod protocol;
mod repl;
mod transport;

use anyhow::Result;
use clap::{Parser, Subcommand};

#[derive(Parser)]
#[command(name = "elle", about = "Elle flight controller CLI")]
struct Cli {
    /// TCP address for RPC responses (RTT up channel 1)
    #[arg(long, default_value = "127.0.0.1:19021", global = true)]
    rx_addr: String,

    /// TCP address for RPC requests (RTT down channel 0)
    #[arg(long, default_value = "127.0.0.1:19022", global = true)]
    tx_addr: String,

    /// Dry-run mode: skip connection, use fake responses (for UI testing)
    #[arg(long, global = true)]
    dry_run: bool,

    #[command(subcommand)]
    command: Option<Commands>,
}

#[derive(Subcommand)]
enum Commands {
    /// Direct probe access mode (single commands, no cargo-embed needed)
    #[command(subcommand)]
    Direct(direct::DirectCommand),
}

#[tokio::main]
async fn main() -> Result<()> {
    let cli = Cli::parse();

    match cli.command {
        None if cli.dry_run => repl::run_dry_run().await,
        None => repl::run_tcp(&cli.rx_addr, &cli.tx_addr).await,
        Some(Commands::Direct(cmd)) => direct::run(cmd).await,
    }
}
