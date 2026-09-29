//! Elle RPC Host CLI
//!
//! Communicate with the Elle flight controller via postcard-RPC over RTT.
//!
//! Modes:
//! - Default: TUI monitoring dashboard with live telemetry
//! - Direct: Single commands via debug probe (for scripting)

mod direct;
mod tui;

use anyhow::Result;
use clap::{Parser, Subcommand};

#[derive(Parser)]
#[command(name = "elle", about = "Elle flight controller CLI")]
struct Cli {
    #[command(subcommand)]
    command: Option<Commands>,
}

#[derive(Subcommand)]
enum Commands {
    /// Single command via debug probe (for scripting)
    #[command(subcommand)]
    Direct(direct::DirectCommand),
}

#[tokio::main]
async fn main() -> Result<()> {
    let cli = Cli::parse();

    match cli.command {
        None => tui::run().await,
        Some(Commands::Direct(cmd)) => direct::run(cmd).await,
    }
}
