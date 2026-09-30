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
    /// MCP server on stdio: holds the probe for an agent (see README)
    Mcp {
        /// Let the agent arm the engines and set throttle (props off!). Without
        /// it, `arm` and `set_throttle` are refused.
        #[arg(long)]
        dangerously_allow_motors: bool,
        /// Longest the engines stay armed before the server disarms, seconds.
        #[arg(long, default_value_t = 60)]
        max_armed_s: u64,
        /// Highest throttle the agent may set, percent.
        #[arg(long, default_value_t = 30, value_parser = clap::value_parser!(u8).range(0..=100))]
        max_throttle: u8,
    },
}

#[tokio::main]
async fn main() -> Result<()> {
    let cli = Cli::parse();

    match cli.command {
        None => tui::run().await,
        Some(Commands::Direct(cmd)) => direct::run(cmd).await,
        Some(Commands::Mcp {
            dangerously_allow_motors,
            max_armed_s,
            max_throttle,
        }) => {
            elle_rpc_host::mcp::serve(elle_rpc_host::mcp::Options {
                allow_motors: dangerously_allow_motors,
                max_armed: std::time::Duration::from_secs(max_armed_s),
                max_throttle_pct: max_throttle,
            })
            .await
        }
    }
}
