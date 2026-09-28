//! Host tests for the ESC link tracker.
//!
//!   cargo test -p elle-control --target x86_64-unknown-linux-gnu --test esc_link

use elle_control::esc_link::{EdtAction, EdtWatch, EscLink, EscLinkEvent};

const SILENT: u32 = 100;

fn feed(link: &mut EscLink, answered: bool, n: u32) -> Vec<EscLinkEvent> {
    (0..n).filter_map(|_| link.update(answered)).collect()
}

#[test]
fn esc_powered_after_boot_appears_once() {
    // Boot configuration went unanswered, then the ESC starts replying.
    let mut link = EscLink::new(SILENT);
    assert!(feed(&mut link, false, 2_000).is_empty());
    assert_eq!(feed(&mut link, true, 50), vec![EscLinkEvent::Appeared]);
    assert!(link.is_up());
}

#[test]
fn configured_at_boot_raises_nothing() {
    let mut link = EscLink::new(SILENT);
    link.mark_up();
    assert!(feed(&mut link, true, 1_000).is_empty());
}

#[test]
fn restart_mid_session_is_lost_then_appeared() {
    let mut link = EscLink::new(SILENT);
    link.mark_up();
    assert!(feed(&mut link, true, 500).is_empty());
    let lost = feed(&mut link, false, 300);
    assert_eq!(lost, vec![EscLinkEvent::Lost]);
    assert!(!link.is_up());
    assert_eq!(feed(&mut link, true, 10), vec![EscLinkEvent::Appeared]);
}

#[test]
fn lost_fires_exactly_at_the_threshold() {
    let mut link = EscLink::new(SILENT);
    link.mark_up();
    assert!(feed(&mut link, false, SILENT - 1).is_empty());
    assert_eq!(link.update(false), Some(EscLinkEvent::Lost));
}

#[test]
fn short_gaps_raise_nothing() {
    let mut link = EscLink::new(SILENT);
    link.mark_up();
    for _ in 0..50 {
        assert!(feed(&mut link, false, SILENT - 1).is_empty());
        assert!(feed(&mut link, true, 1).is_empty());
    }
    assert!(link.is_up());
}

#[test]
fn permanently_silent_esc_does_not_spam() {
    // A non-bidirectional ESC never answers: no events at all.
    let mut link = EscLink::new(SILENT);
    assert!(feed(&mut link, false, 100_000).is_empty());
    // And once lost, staying silent raises nothing more.
    let mut link = EscLink::new(SILENT);
    link.mark_up();
    assert_eq!(feed(&mut link, false, 100_000), vec![EscLinkEvent::Lost]);
}

const CONFIRM: u32 = 5_000;
const RETRIES: u8 = 3;

/// Feed `n` answered frames, `edt` saying whether each carried EDT.
fn edt_feed(w: &mut EdtWatch, edt: bool, stopped: bool, n: u32) -> Vec<EdtAction> {
    (0..n)
        .filter_map(|_| w.update(true, edt, stopped))
        .collect()
}

#[test]
fn edt_arriving_needs_nothing() {
    let mut w = EdtWatch::new(CONFIRM, RETRIES);
    assert!(edt_feed(&mut w, false, true, 100).is_empty());
    assert!(edt_feed(&mut w, true, true, 1).is_empty());
    assert!(w.edt_seen());
    assert!(edt_feed(&mut w, false, true, 100_000).is_empty());
}

#[test]
fn missing_edt_asks_for_a_reconfigure_then_rechecks() {
    let mut w = EdtWatch::new(CONFIRM, RETRIES);
    assert!(edt_feed(&mut w, false, true, CONFIRM - 1).is_empty());
    assert_eq!(
        edt_feed(&mut w, false, true, 1),
        vec![EdtAction::Reconfigure]
    );
    w.configured();
    // EDT takes this time.
    assert!(edt_feed(&mut w, false, true, 200).is_empty());
    assert!(edt_feed(&mut w, true, true, 1).is_empty());
    assert!(edt_feed(&mut w, false, true, 100_000).is_empty());
}

#[test]
fn gives_up_once_after_the_retries() {
    let mut w = EdtWatch::new(CONFIRM, RETRIES);
    let mut actions = Vec::new();
    for _ in 0..(RETRIES + 3) {
        let a = edt_feed(&mut w, false, true, CONFIRM);
        if a.contains(&EdtAction::Reconfigure) {
            w.configured();
        }
        actions.extend(a);
    }
    let reconfigures = actions
        .iter()
        .filter(|a| **a == EdtAction::Reconfigure)
        .count();
    assert_eq!(reconfigures, RETRIES as usize);
    assert_eq!(actions.last(), Some(&EdtAction::GiveUp));
    assert_eq!(
        actions.iter().filter(|a| **a == EdtAction::GiveUp).count(),
        1
    );
}

#[test]
fn waits_for_the_motor_to_stop() {
    let mut w = EdtWatch::new(CONFIRM, RETRIES);
    // Running: counts, but never asks for settings commands.
    assert!(edt_feed(&mut w, false, false, CONFIRM * 3).is_empty());
    // First stopped frame after that asks.
    assert_eq!(
        edt_feed(&mut w, false, true, 1),
        vec![EdtAction::Reconfigure]
    );
}

#[test]
fn unanswered_frames_do_not_count() {
    let mut w = EdtWatch::new(CONFIRM, RETRIES);
    let none: Vec<_> = (0..CONFIRM * 3)
        .filter_map(|_| w.update(false, false, true))
        .collect();
    assert!(none.is_empty());
}

#[test]
fn restart_clears_the_retries() {
    let mut w = EdtWatch::new(CONFIRM, RETRIES);
    for _ in 0..RETRIES {
        assert_eq!(
            edt_feed(&mut w, false, true, CONFIRM),
            vec![EdtAction::Reconfigure]
        );
        w.configured();
    }
    w.reset();
    // A fresh ESC gets its retries back instead of an immediate give-up.
    assert_eq!(
        edt_feed(&mut w, false, true, CONFIRM),
        vec![EdtAction::Reconfigure]
    );
}
