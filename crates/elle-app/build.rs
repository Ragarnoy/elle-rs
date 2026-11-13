//! Records `git describe --always --dirty` as `ELLE_GIT_DESCRIBE` for the
//! `GetBuildInfo` RPC and the ULog header's `ver_sw_git` ("unknown" outside a
//! git checkout).

use std::process::Command;

fn main() {
    let describe = Command::new("git")
        .args(["describe", "--always", "--dirty", "--abbrev=10"])
        .output()
        .ok()
        .filter(|o| o.status.success())
        .and_then(|o| String::from_utf8(o.stdout).ok())
        .map_or_else(|| "unknown".to_string(), |s| s.trim().to_string());
    println!("cargo:rustc-env=ELLE_GIT_DESCRIBE={describe}");
    // Re-run when HEAD moves or the index changes (a commit, a checkout, an edit
    // staged), and when any firmware source or manifest changes, so an unstaged
    // edit (a gain changed by hand) still shows `-dirty`.
    for p in [
        "../../.git/HEAD",
        "../../.git/index",
        "../../crates",
        "../../drivers",
        "../../Cargo.toml",
        "../../Cargo.lock",
    ] {
        println!("cargo:rerun-if-changed={p}");
    }
}
