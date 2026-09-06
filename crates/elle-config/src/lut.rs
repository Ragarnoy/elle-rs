#![allow(clippy::inline_always)]
use crate::*;

/// Const-compatible clamp for i32 (stable Rust `i32::clamp` is not const).
const fn const_clamp_i32(v: i32, lo: i32, hi: i32) -> i32 {
    if v < lo {
        lo
    } else if v > hi {
        hi
    } else {
        v
    }
}

/// Compute reduction percentage, clamped to `[0, max_percent]`.
const fn compute_reduction(amount: i32, max_range: i32, max_percent: i32) -> i32 {
    let raw = if max_range > 0 {
        amount * max_percent / max_range
    } else {
        max_percent
    };
    if raw < max_percent { raw } else { max_percent }
}

/// RC value range (0-2047, so we need 2048 entries)
pub const RC_MAX_VALUE: usize = 2047;
pub const RC_LUT_SIZE: usize = RC_MAX_VALUE + 1;

// Differential thrust LUT size - covers full RC range
pub const DIFF_LUT_SIZE: usize = RC_LUT_SIZE;

/// DShot throttle value where motors start spinning (equivalent to ENGINE_START_PULSE_US in µs space)
const DSHOT_START_THROTTLE: u16 = ((ENGINE_START_PULSE_US - ENGINE_MIN_PULSE_US)
    * DSHOT_THROTTLE_MAX as u32
    / (ENGINE_MAX_PULSE_US - ENGINE_MIN_PULSE_US)) as u16;

/// Const function to generate throttle curve lookup table at compile time.
/// Output is in DShot space (0-1999) instead of µs.
const fn generate_throttle_lut() -> [u16; RC_LUT_SIZE] {
    let mut lut = [0u16; RC_LUT_SIZE];
    let mut i = 0;

    while i < RC_LUT_SIZE {
        let rc = i as u32;

        let value = if rc <= THROTTLE_DEADZONE {
            0
        } else if rc <= THROTTLE_START_POINT {
            let progress =
                (rc - THROTTLE_DEADZONE) * 100 / (THROTTLE_START_POINT - THROTTLE_DEADZONE);
            (DSHOT_START_THROTTLE as u32 * progress / 100) as u16
        } else {
            let range = 2047 - THROTTLE_START_POINT;
            let position = rc - THROTTLE_START_POINT;
            (DSHOT_START_THROTTLE as u32
                + position * (DSHOT_THROTTLE_MAX as u32 - DSHOT_START_THROTTLE as u32) / range)
                as u16
        };

        lut[i] = value;
        i += 1;
    }

    lut
}

/// Const function to generate servo pulse lookup table at compile time
const fn generate_servo_lut(min_us: u32, max_us: u32) -> [u32; RC_LUT_SIZE] {
    let mut lut = [0u32; RC_LUT_SIZE];
    let mut i = 0;

    while i < RC_LUT_SIZE {
        let rc = i as u32;
        let value = min_us + (rc * (max_us - min_us) / 2047);
        lut[i] = value;
        i += 1;
    }

    lut
}

/// Const function to generate normalized value lookup table
/// Values are (rc - center), clamped to [-1024, 1024]; divide by 1024.0 for [-1.0, 1.0]
const fn generate_normalized_lut(center: u16) -> [i32; RC_LUT_SIZE] {
    let mut lut = [0i32; RC_LUT_SIZE];
    let mut i = 0;

    while i < RC_LUT_SIZE {
        let rc = i as i32;
        let normalized_fp = rc - center as i32;

        // Clamp to -1024 to 1024 (representing -1.0 to 1.0)
        lut[i] = const_clamp_i32(normalized_fp, -1024, 1024);

        i += 1;
    }

    lut
}

/// Generate differential thrust multiplier LUT (legacy support)
const fn generate_differential_lut() -> [(u32, u32); DIFF_LUT_SIZE] {
    let mut lut = [(100u32, 100u32); DIFF_LUT_SIZE];
    let mut i = 0;

    while i < DIFF_LUT_SIZE {
        let ch4_value = i as u16;

        let (left, right) = if ch4_value >= DIFF_NEUTRAL_MIN && ch4_value <= DIFF_NEUTRAL_MAX {
            (100, 100)
        } else if ch4_value < DIFF_NEUTRAL_MIN {
            let amount = (DIFF_NEUTRAL_MIN - ch4_value) as i32;
            let max_range = (DIFF_NEUTRAL_MIN - 300) as i32;
            let reduction = compute_reduction(amount, max_range, DIFF_MAX_PERCENT);
            ((100 - reduction) as u32, 100)
        } else {
            let amount = (ch4_value - DIFF_NEUTRAL_MAX) as i32;
            let max_range = (1700 - DIFF_NEUTRAL_MAX) as i32;
            let reduction = compute_reduction(amount, max_range, DIFF_MAX_PERCENT);
            (100, (100 - reduction) as u32)
        };

        lut[i] = (left, right);
        i += 1;
    }

    lut
}

/// Generate yaw differential factors LUT (for mixing mode)
const fn generate_yaw_differential_lut() -> [(i32, i32); RC_LUT_SIZE] {
    let mut lut = [(1024i32, 1024i32); RC_LUT_SIZE]; // 1024 = 1.0 in fixed point
    let mut i = 0;

    while i < RC_LUT_SIZE {
        let rc_value = i as u16;

        let center = RC_CENTER as i32;
        let normalized_fp = rc_value as i32 - center;
        let yaw_input_fp = const_clamp_i32(normalized_fp, -1024, 1024);

        // Apply YAW_TO_DIFF_GAIN (assuming 1.0 for now, can be adjusted)
        let yaw_factor_fp = yaw_input_fp; // * YAW_TO_DIFF_GAIN in fixed point

        let (left_mult_fp, right_mult_fp) = if yaw_factor_fp > 0 {
            // Right turn: reduce left engine
            let reduction = (yaw_factor_fp * 205) / 1024; // 0.2 * 1024 = ~205 in fixed point
            let reduction = if reduction > 205 { 205 } else { reduction };
            (1024 - reduction, 1024)
        } else if yaw_factor_fp < 0 {
            // Left turn: reduce right engine
            let reduction = ((-yaw_factor_fp) * 205) / 1024; // 0.2 * 1024 = ~205 in fixed point
            let reduction = if reduction > 205 { 205 } else { reduction };
            (1024, 1024 - reduction)
        } else {
            (1024, 1024) // No yaw input
        };

        // Clamp to 0.8-1.0 range (819 to 1024 in fixed point)
        let left_final = const_clamp_i32(left_mult_fp, 819, 1024);
        let right_final = const_clamp_i32(right_mult_fp, 819, 1024);

        lut[i] = (left_final, right_final);
        i += 1;
    }

    lut
}

// Pre-computed lookup tables - all generated at compile time
pub static THROTTLE_LUT: [u16; RC_LUT_SIZE] = generate_throttle_lut();
pub static SERVO_LUT: [u32; RC_LUT_SIZE] =
    generate_servo_lut(SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
pub static ENGINE_LUT: [u32; RC_LUT_SIZE] =
    generate_servo_lut(ENGINE_MIN_PULSE_US, ENGINE_MAX_PULSE_US);

// Single normalized lookup table — all channels share RC_CENTER after CRSF scaling
pub static NORMALIZED_LUT: [i32; RC_LUT_SIZE] = generate_normalized_lut(RC_CENTER);

// Differential thrust LUTs
pub static DIFFERENTIAL_LEGACY_LUT: [(u32, u32); DIFF_LUT_SIZE] = generate_differential_lut();
pub static YAW_DIFFERENTIAL_LUT: [(i32, i32); RC_LUT_SIZE] = generate_yaw_differential_lut();

/// Ultra-fast throttle curve lookup - single array access (returns DShot 0-1999)
#[must_use]
#[inline(always)]
pub fn throttle_curve_lut(rc_value: u16) -> u16 {
    unsafe {
        // SAFETY: We clamp the index to valid range
        *THROTTLE_LUT.get_unchecked((rc_value as usize).min(RC_MAX_VALUE))
    }
}

/// Ultra-fast servo pulse lookup
#[must_use]
#[inline(always)]
pub fn rc_to_pulse_lut(rc_value: u16) -> u32 {
    unsafe {
        // SAFETY: We clamp the index to valid range
        *SERVO_LUT.get_unchecked((rc_value as usize).min(RC_MAX_VALUE))
    }
}

/// Ultra-fast engine pulse lookup (for arming logic - linear mapping)
#[must_use]
#[inline(always)]
pub fn rc_to_engine_pulse_lut(rc_value: u16) -> u32 {
    unsafe {
        // SAFETY: We clamp the index to valid range
        *ENGINE_LUT.get_unchecked((rc_value as usize).min(RC_MAX_VALUE))
    }
}

/// Ultra-fast normalized value lookup (all axes share the same center)
#[must_use]
#[inline(always)]
pub fn rc_to_normalized(rc_value: u16) -> f32 {
    unsafe {
        // SAFETY: We clamp the index to valid range
        let fixed_point = *NORMALIZED_LUT.get_unchecked((rc_value as usize).min(RC_MAX_VALUE));
        fixed_point as f32 / 1024.0
    }
}

/// Ultra-fast differential thrust calculation (legacy)
#[must_use]
#[inline(always)]
pub fn calculate_differential_lut(ch4_value: u16) -> (u32, u32) {
    unsafe {
        // SAFETY: We clamp the index to valid range
        *DIFFERENTIAL_LEGACY_LUT.get_unchecked((ch4_value as usize).min(RC_MAX_VALUE))
    }
}

/// Ultra-fast yaw differential factors (mixing mode)
#[must_use]
#[inline(always)]
pub fn calculate_yaw_differential_lut(yaw_rc: u16) -> (f32, f32) {
    unsafe {
        // SAFETY: We clamp the index to valid range
        let (left_fp, right_fp) =
            *YAW_DIFFERENTIAL_LUT.get_unchecked((yaw_rc as usize).min(RC_MAX_VALUE));
        (left_fp as f32 / 1024.0, right_fp as f32 / 1024.0)
    }
}

/// Convert raw RC channels to normalized control inputs using LUTs
#[must_use]
#[inline(always)]
pub fn channels_to_normalized_lut(channels: &[u16]) -> (f32, f32, f32, f32) {
    (
        rc_to_normalized(channels[ROLL_CH]) * ROLL_INVERT,
        rc_to_normalized(channels[PITCH_CH]) * PITCH_INVERT,
        rc_to_normalized(channels[YAW_CH]) * YAW_INVERT,
        (channels[THROTTLE_CH] as f32) / 2047.0,
    )
}

/// Apply differential thrust using pre-computed values (mixing mode, DShot space)
#[must_use]
#[inline(always)]
pub fn apply_differential_thrust_lut(base_thrust: u16, yaw_rc: u16) -> (u16, u16) {
    if base_thrust == 0 {
        return (0, 0);
    }
    let (left_mult, right_mult) = calculate_yaw_differential_lut(yaw_rc);
    let left = ((base_thrust as f32 * left_mult) as u16).min(DSHOT_THROTTLE_MAX);
    let right = ((base_thrust as f32 * right_mult) as u16).min(DSHOT_THROTTLE_MAX);
    (left, right)
}

// ============================================================================
// Governor feedforward LUT — eRPM → DShot estimate
// ============================================================================

/// Measured eRPM→DShot mapping (averaged from both engines under EDF load, 4S).
/// Pairs are (eRPM, DShot), sorted by eRPM.
/// Data from rpm_range sweep test with 200-sample settle + 500-sample measurement per step.
#[cfg(not(feature = "platform-dart"))]
const GOVERNOR_FF_TABLE: [(u32, u16); 11] = [
    (6_184, 48),     // avg(915,852) RPM × 7
    (19_177, 148),   // avg(2764,2715) RPM × 7
    (37_044, 298),   // avg(5314,5270) RPM × 7
    (53_991, 448),   // avg(7746,7680) RPM × 7
    (70_508, 598),   // avg(10101,10044) RPM × 7
    (87_777, 748),   // avg(12584,12495) RPM × 7
    (103_100, 898),  // avg(14794,14663) RPM × 7
    (116_284, 1048), // avg(16662,16562) RPM × 7
    (127_932, 1198), // avg(18338,18214) RPM × 7
    (140_284, 1348), // avg(20121,19960) RPM × 7
    (149_800, 1473), // right engine saturation (MAX_ERPM)
                     // Above this the right engine is voltage-limited; left can go slightly higher
                     // but governor caps at MAX_ERPM to keep thrust symmetric.
];

/// Governor feedforward: estimate DShot output for a target eRPM using measured LUT.
/// Piecewise linear interpolation between measured data points.
/// Returns DShot 0–1999.
#[cfg(not(feature = "platform-dart"))]
#[must_use]
pub fn governor_feedforward(target_erpm: u32) -> u16 {
    if target_erpm == 0 {
        return 0;
    }

    // Below the first entry: extrapolate linearly from origin
    if target_erpm <= GOVERNOR_FF_TABLE[0].0 {
        return ((target_erpm * GOVERNOR_FF_TABLE[0].1 as u32) / GOVERNOR_FF_TABLE[0].0) as u16;
    }

    // Above the last entry: clamp to max DShot
    let last = GOVERNOR_FF_TABLE[GOVERNOR_FF_TABLE.len() - 1];
    if target_erpm >= last.0 {
        return last.1.min(DSHOT_THROTTLE_MAX);
    }

    // Linear search (table is small, ~12 entries — faster than binary search on MCU)
    for i in 1..GOVERNOR_FF_TABLE.len() {
        if target_erpm <= GOVERNOR_FF_TABLE[i].0 {
            let (erpm_lo, dshot_lo) = GOVERNOR_FF_TABLE[i - 1];
            let (erpm_hi, dshot_hi) = GOVERNOR_FF_TABLE[i];
            let range_erpm = erpm_hi - erpm_lo;
            let range_dshot = dshot_hi as u32 - dshot_lo as u32;
            let offset = target_erpm - erpm_lo;
            return (dshot_lo as u32 + offset * range_dshot / range_erpm) as u16;
        }
    }

    last.1
}

/// Measured eRPM→DShot mapping for dart (500-sample per step, rpm_range sweep).
/// RPM × 7 (14-pole assumed). Covers DShot 48–1748; RPM plateaus 1748–1798 and
/// declines above that (prop stall), so the table stops at the last rising point.
#[cfg(feature = "platform-dart")]
const GOVERNOR_FF_TABLE: [(u32, u16); 18] = [
    (2_534, 48),    //    362 RPM
    (9_569, 148),   //  1,367 RPM
    (17_997, 248),  //  2,571 RPM
    (25_816, 348),  //  3,688 RPM
    (33_096, 448),  //  4,728 RPM
    (39_690, 548),  //  5,670 RPM
    (46_270, 648),  //  6,610 RPM
    (53_039, 748),  //  7,577 RPM
    (57_988, 848),  //  8,284 RPM
    (63_126, 948),  //  9,018 RPM
    (68_810, 1048), //  9,830 RPM
    (73_325, 1148), // 10,475 RPM
    (77_287, 1248), // 11,041 RPM
    (80_808, 1348), // 11,544 RPM
    (84_462, 1448), // 12,066 RPM
    (87_955, 1548), // 12,565 RPM
    (91_084, 1648), // 13,012 RPM
    (93_968, 1748), // 13,424 RPM — last rising point; governor caps here
];

/// Governor feedforward for dart: piecewise linear interpolation over measured LUT.
/// Returns DShot 0–1748 (capped below the RPM plateau to avoid the prop stall region).
#[cfg(feature = "platform-dart")]
#[must_use]
pub fn governor_feedforward(target_erpm: u32) -> u16 {
    if target_erpm == 0 {
        return 0;
    }

    if target_erpm <= GOVERNOR_FF_TABLE[0].0 {
        return ((target_erpm * GOVERNOR_FF_TABLE[0].1 as u32) / GOVERNOR_FF_TABLE[0].0) as u16;
    }

    let last = GOVERNOR_FF_TABLE[GOVERNOR_FF_TABLE.len() - 1];
    if target_erpm >= last.0 {
        return last.1; // 1748 — never enter prop stall region
    }

    for i in 1..GOVERNOR_FF_TABLE.len() {
        if target_erpm <= GOVERNOR_FF_TABLE[i].0 {
            let (erpm_lo, dshot_lo) = GOVERNOR_FF_TABLE[i - 1];
            let (erpm_hi, dshot_hi) = GOVERNOR_FF_TABLE[i];
            let range_erpm = erpm_hi - erpm_lo;
            let range_dshot = dshot_hi as u32 - dshot_lo as u32;
            let offset = target_erpm - erpm_lo;
            return (dshot_lo as u32 + offset * range_dshot / range_erpm) as u16;
        }
    }

    last.1
}

/// Apply differential thrust using pre-computed values (legacy, DShot space)
#[must_use]
#[inline(always)]
pub fn apply_differential_lut(base_thrust: u16, ch4_value: u16) -> (u16, u16) {
    if base_thrust == 0 {
        return (0, 0);
    }
    let (left_mult, right_mult) = calculate_differential_lut(ch4_value);
    let left = ((base_thrust as u32 * left_mult / 100) as u16).min(DSHOT_THROTTLE_MAX);
    let right = ((base_thrust as u32 * right_mult / 100) as u16).min(DSHOT_THROTTLE_MAX);
    (left, right)
}

/// DShot value of the last `GOVERNOR_FF_TABLE` entry — the highest output the
/// feedforward will ever produce. `GOVERNOR_DSHOT_MAX` must equal this (asserted
/// in `lib.rs`), or the PI correction could push past where the table refuses to go.
#[must_use]
pub const fn governor_ff_max_dshot() -> u16 {
    GOVERNOR_FF_TABLE[GOVERNOR_FF_TABLE.len() - 1].1
}
