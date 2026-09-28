//! Host tests for the ESC link tracker.
//!
//!   cargo test -p elle-control --target x86_64-unknown-linux-gnu --test esc_link

use elle_control::esc_link::{EscLink, EscLinkEvent};

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
