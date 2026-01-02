//! Interactive REPL mode

use anyhow::Result;
use console::style;
use dialoguer::{theme::ColorfulTheme, Confirm, FuzzySelect, Input};

use crate::commands::CommandHandler;
use crate::transport::{MockTransport, TcpTransport, Transport};

const MENU_COMMANDS: &[&str] = &[
    "Ping        - Check device connectivity",
    "Status      - Get system status",
    "Attitude    - Get current attitude",
    "Version     - Get firmware version",
    "Perf        - Get performance stats",
    "Arm         - Arm motors",
    "Disarm      - Disarm motors",
    "Throttle    - Set throttle (0-100%)",
    "Elevon      - Set elevon positions",
    "Stop        - EMERGENCY STOP",
    "Quit        - Exit",
];

fn validate_percent(v: &u8) -> Result<(), &'static str> {
    if *v <= 100 {
        Ok(())
    } else {
        Err("Must be 0-100")
    }
}

fn validate_elevon(v: &i8) -> Result<(), &'static str> {
    if *v >= -100 && *v <= 100 {
        Ok(())
    } else {
        Err("Must be -100 to 100")
    }
}

async fn run_command_loop<T: Transport>(transport: &mut T) -> Result<()> {
    let theme = ColorfulTheme::default();

    while let Ok(Some(selection)) = FuzzySelect::with_theme(&theme)
        .with_prompt("Command")
        .items(MENU_COMMANDS)
        .default(0)
        .interact_opt()
    {
        let mut h = CommandHandler::new(transport);

        let result = match selection {
            0 => h.ping().await,
            1 => h.status().await,
            2 => h.attitude().await,
            3 => h.version().await,
            4 => h.perf().await,
            5 => {
                if Confirm::with_theme(&theme)
                    .with_prompt("Arm motors?")
                    .default(false)
                    .interact()
                    .unwrap_or(false)
                {
                    h.arm().await
                } else {
                    Ok(())
                }
            }
            6 => h.disarm().await,
            7 => {
                match Input::<u8>::with_theme(&theme)
                    .with_prompt("Throttle %")
                    .validate_with(validate_percent)
                    .interact()
                {
                    Ok(pct) => h.throttle(pct).await,
                    Err(_) => Ok(()),
                }
            }
            8 => {
                let Ok(left) = Input::<i8>::with_theme(&theme)
                    .with_prompt("Left")
                    .validate_with(validate_elevon)
                    .interact()
                else {
                    continue;
                };
                let Ok(right) = Input::<i8>::with_theme(&theme)
                    .with_prompt("Right")
                    .validate_with(validate_elevon)
                    .interact()
                else {
                    continue;
                };
                h.elevon(left, right).await
            }
            9 => {
                if Confirm::with_theme(&theme)
                    .with_prompt(format!("{}", style("EMERGENCY STOP?").red().bold()))
                    .default(false)
                    .interact()
                    .unwrap_or(false)
                {
                    h.emergency_stop().await
                } else {
                    Ok(())
                }
            }
            10 => {
                println!("{}", style("Goodbye!").dim());
                return Ok(());
            }
            _ => Ok(()),
        };

        if let Err(e) = result {
            println!("{}: {e}", style("Error").red());
        }
        println!();
    }

    println!("{}", style("Goodbye!").dim());
    Ok(())
}

/// Run interactive REPL with TCP transport
pub async fn run_tcp(rx_addr: &str, tx_addr: &str) -> Result<()> {
    println!("{}", style("Elle CLI").bold().cyan());
    let mut transport = TcpTransport::connect_with_retry(rx_addr, tx_addr).await;
    println!("{}\n", style("Connected!").green());

    loop {
        if let Err(e) = run_command_loop(&mut transport).await {
            if e.to_string().contains("Connection") || e.to_string().contains("closed") {
                println!("{}", style("Reconnecting...").yellow());
                transport = TcpTransport::connect_with_retry(rx_addr, tx_addr).await;
                println!("{}", style("Reconnected!").green());
                continue;
            }
            return Err(e);
        }
        return Ok(());
    }
}

/// Run interactive REPL with mock transport (dry-run mode)
pub async fn run_dry_run() -> Result<()> {
    println!("{}", style("Elle CLI").bold().cyan());
    println!("{}\n", style("[DRY-RUN MODE - fake responses]").yellow());

    let mut transport = MockTransport;
    run_command_loop(&mut transport).await
}
