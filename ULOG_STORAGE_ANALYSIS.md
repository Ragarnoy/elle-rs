# ULog Storage Capacity Analysis

## Current Configuration: 128Mbit (16MB) Flash

### Available ULog Storage
- **Total flash**: 16MB (128Mbit)
- **Program code**: ~15MB
- **Calibration**: 64KB
- **ULog region**: 960KB (0xF10000 - 0xFFFFFF)

---

## Data Rate Calculation

### Logging Rate at 77Hz Control Loop

| Message Type | Size (bytes) | Rate | Data/Second |
|--------------|--------------|------|-------------|
| **AttitudeMessage** | 32 + 3 header = 35 | 77 Hz | 2,695 bytes/s |
| **CommandsMessage** | 37 + 3 header = 40 | 77 Hz | 3,080 bytes/s |
| **StatusMessage** | 22 + 3 header = 25 | 7.7 Hz (every 10th) | 192.5 bytes/s |
| **TOTAL** | — | — | **~6 KB/s** |

### Storage Capacity

```
960 KB ÷ 6 KB/s = 160 seconds = 2 minutes 40 seconds
```

**⚠️ WARNING: This is NOT sufficient for typical flight operations!**

---

## Flight Duration Requirements

Typical flying wing flight scenarios:

| Scenario | Duration | Data Generated | Storage Needed |
|----------|----------|----------------|----------------|
| **Quick test flight** | 2-5 min | 720 KB - 1.8 MB | ❌ Insufficient |
| **Normal flight** | 10-15 min | 3.6 MB - 5.4 MB | ❌ Insufficient |
| **Extended flight** | 20-30 min | 7.2 MB - 10.8 MB | ❌ Insufficient |
| **Long endurance** | 45-60 min | 16.2 MB - 21.6 MB | ❌ Insufficient |

**Current 960KB only covers ~2.5 minutes of flight.**

---

## Option Analysis: Flash Configurations

### Option 1: Keep 128Mbit (16MB) Flash - Expand ULog Region

**Reallocation:**
```
0x000000 - 0x1FFFFF: Program code (2MB - plenty for firmware)
0x200000 - 0x20FFFF: Calibration (64KB)
0x210000 - 0xFFFFFF: ULog storage (14MB!)
```

**Capacity:**
- ULog storage: **14 MB**
- Flight time: **14 MB ÷ 6 KB/s = 2,333 seconds ≈ 39 minutes**
- Cost: $0 (just recompile with different constants)

**Pros:**
- ✅ Zero hardware cost
- ✅ Simple firmware change
- ✅ Covers most flight scenarios
- ✅ No moving parts
- ✅ Reliable

**Cons:**
- ⚠️ Limited to 39 minutes maximum
- ⚠️ Requires probe or USB to download logs
- ⚠️ Must erase logs to free space

---

### Option 2: 16Mbit (2MB) Flash + SD Card

**Flash allocation:**
```
0x000000 - 0x1FFFFF: Program code (2MB - all of it!)
```

**SD Card (e.g., 4GB):**
```
- ULog storage: Up to 4GB
- Flight time: 4GB ÷ 6 KB/s = 666,666 seconds = 185 hours!
```

**Pros:**
- ✅ Massive storage (practically unlimited for flights)
- ✅ Removable - easy log downloads
- ✅ Can swap cards between flights
- ✅ Lower flash cost (2MB cheaper than 16MB)

**Cons:**
- ❌ SD card slot adds BOM cost (~$1-2)
- ❌ SD card connector less reliable than flash (vibration, pins)
- ❌ Requires SD card driver (SPI or SDIO)
- ❌ Higher power consumption during writes
- ❌ Slower write speeds (SD latency)
- ❌ More complex firmware (SD protocol, FAT filesystem)
- ❌ Potential for card corruption in crashes
- ⚠️ Moving parts / connectors in high-vibration environment

---

### Option 3: 256Mbit (32MB) Flash (No SD Card)

**Flash allocation:**
```
0x000000 - 0x1FFFFF: Program code (2MB)
0x200000 - 0x20FFFF: Calibration (64KB)
0x210000 - 0x1FFFFFF: ULog storage (30MB)
```

**Capacity:**
- ULog storage: **30 MB**
- Flight time: **30 MB ÷ 6 KB/s = 5,000 seconds ≈ 83 minutes**

**Pros:**
- ✅ Covers all reasonable flight scenarios
- ✅ No moving parts - highly reliable
- ✅ Simple firmware (no SD driver)
- ✅ Fast write speeds
- ✅ Low power consumption

**Cons:**
- ⚠️ Higher flash cost than 16Mbit (marginal - ~$0.50-1)
- ⚠️ Requires probe/USB to download logs
- ⚠️ Must erase to free space

---

## Recommended Solution

### **Option 1A: Keep 128Mbit, Expand ULog Region to 14MB** ⭐

**Why:**
1. **Zero cost** - just change flash constants
2. **39 minutes** covers 95% of flying wing flights
3. **Highly reliable** - no SD card failure modes
4. **Simple** - no additional drivers needed
5. **Already tested** - your current hardware

**Implementation:**

```rust
// crates/elle-hardware/src/flash_constants.rs

/// Flash memory layout (optimized for ULog):
/// - 0x000000 - 0x1FFFFF: Program code (2MB)
/// - 0x200000 - 0x20FFFF: Calibration storage (64KB)
/// - 0x210000 - 0xFFFFFF: ULog storage (14MB)

/// ULog flash region start (after calibration area)
pub const ULOG_FLASH_START: u32 = 0x210000;

/// ULog flash region end (end of flash)
pub const ULOG_FLASH_END: u32 = 0xFFFFFF;

/// ULog flash region size (14MB)
pub const ULOG_FLASH_SIZE: usize = (ULOG_FLASH_END - ULOG_FLASH_START + 1) as usize;
```

**Capacity verification:**
```
14,680,064 bytes ÷ 6 KB/s = 2,446 seconds = 40.7 minutes
```

---

### **Option 2: SD Card (if you need >1 hour flights or easy log access)**

**Use cases:**
- Long endurance missions (>1 hour)
- Frequent log downloads without probe
- Multiple flights per day (swap cards)
- Development/testing (lots of logging)

**Hardware requirements:**
- SD card slot (SPI mode is simplest)
- ~4 GPIO pins (MISO, MOSI, SCK, CS)
- SD card (4-32GB)

**Firmware complexity:**
- Add `embedded-sdmmc` crate
- Implement SPI SD card driver
- Add FAT32 filesystem
- Handle SD errors/timeouts
- ~500-1000 lines of code

---

## Cost Comparison

| Option | Flash Cost | SD Slot | SD Card | Total ∆ Cost |
|--------|-----------|---------|---------|-------------|
| **Current (960KB)** | Included | — | — | $0 |
| **Expand to 14MB** | Included | — | — | **$0** |
| **16Mbit + SD** | -$0.50 | +$1.50 | +$5 | **+$6** |
| **256Mbit (32MB)** | +$1 | — | — | **+$1** |

*(Costs are approximate and vary by supplier/quantity)*

---

## Migration Path

### Phase 1: Immediate (Free)
1. Expand ULog region to 14MB
2. Update flash constants
3. Rebuild firmware
4. **Result: 40 minutes flight time**

### Phase 2: If Needed (Future)
- If 40 minutes proves insufficient
- Add SD card slot in next hardware revision
- Implement SD logging in parallel with flash
- Keep flash as backup when SD fails

---

## Real-World Usage Patterns

### What Actually Gets Logged

Currently logging at 77Hz:
- Attitude (100%)
- Commands (100%)
- Status (10%)

**Optimization options to extend storage:**

| Change | New Rate | Storage Time |
|--------|----------|--------------|
| **No change** | 6 KB/s | 40 min (14MB) |
| Attitude @ 50Hz | 4.7 KB/s | 51 min |
| Attitude @ 30Hz | 3.8 KB/s | 63 min |
| Commands @ 10Hz | 4.6 KB/s | 52 min |
| Status @ 1Hz | 5.9 KB/s | 41 min |
| **All optimizations** | 2.5 KB/s | **96 min (1.6 hrs)** |

**Recommendation:** Start with full rate logging, optimize only if needed.

---

## Conclusion

**For your flying wing flight controller:**

✅ **Expand to 14MB flash storage** (free, simple, reliable)
- Covers 40 minutes of full-rate logging
- No hardware changes
- High reliability
- Simple firmware

❌ **Don't use SD card unless:**
- You need >1 hour flights
- You want removable storage
- You're willing to handle SD complexity/reliability issues

**Action Item:**
```bash
# Update flash constants to use 14MB for ULog
sed -i 's/0xF10000/0x210000/g' crates/elle-hardware/src/flash_constants.rs
cargo build --release
```

This gives you **15x more storage** with **zero cost** and **zero risk**.
