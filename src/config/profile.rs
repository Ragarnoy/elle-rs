use crate::hardware::imu::CalibrationLevels;
use bno055::BNO055_CALIB_SIZE;

pub const FLASH_SIZE: usize = 16 * 1024 * 1024; // 16MB total flash (128 Mbit)
pub const CALIBRATION_FLASH_OFFSET: u32 = 0xF00000; // 15MB offset (1MB from end for safety)
pub const CALIBRATION_MAGIC: u32 = 0x42CE55CA; // "BNO55CAL" as hex - valid u32
pub const CALIBRATION_VERSION: u32 = 1; // Version for future compatibility

// Flash operation requests and responses
#[derive(Clone, Copy, Debug)]
pub enum FlashRequest {
    LoadCalibration,
    SaveCalibration {
        profile_data: [u8; BNO055_CALIB_SIZE],
        quality: CalibrationLevels,
        timestamp: u64,
    },
}

#[derive(Clone, Copy, Debug)]
pub enum FlashResponse {
    LoadSuccess([u8; BNO055_CALIB_SIZE]),
    LoadFailed,
    SaveSuccess,
    SaveFailed,
}

// Calibration storage structure (must be u32-aligned for flash API)
// Total size is padded to match PAGE_SIZE (256 bytes) for optimal flash operations
#[repr(C)]
#[derive(Clone, Copy)]
pub struct StoredCalibration {
    pub magic: u32,
    pub version: u32,
    pub timestamp_lo: u32,      // Lower 32 bits of timestamp
    pub timestamp_hi: u32,      // Upper 32 bits of timestamp
    pub quality_hash: u32,      // Simple hash of calibration quality levels
    pub profile_data: [u32; 6], // BNO055 calibration profile as u32 words (24 bytes, padded)
    pub checksum: u32,          // Simple checksum for data integrity
    pub padding: [u32; 52],     // Padding to make the total size 256 bytes (PAGE_SIZE)
}

impl StoredCalibration {
    pub fn new(profile_bytes: &[u8], quality: &CalibrationLevels, timestamp: u64) -> Self {
        // Convert calibration bytes to u32 array for flash storage
        let mut profile_data = [0u32; 6];
        let profile_u8_slice = unsafe {
            core::slice::from_raw_parts_mut(
                profile_data.as_mut_ptr() as *mut u8,
                24, // 6 * 4 bytes
            )
        };

        // Copy profile data (22 bytes) into u32 array, zero-padded
        let copy_len = profile_bytes.len().min(22);
        profile_u8_slice[..copy_len].copy_from_slice(&profile_bytes[..copy_len]);

        let quality_hash = CalibrationLevels::hash_quality(quality);
        let mut cal = Self {
            magic: CALIBRATION_MAGIC,
            version: CALIBRATION_VERSION,
            timestamp_lo: timestamp as u32,
            timestamp_hi: (timestamp >> 32) as u32,
            quality_hash,
            profile_data,
            checksum: 0,
            padding: [0u32; 52], // Initialize padding with zeros
        };

        cal.checksum = cal.calculate_checksum();
        cal
    }

    pub fn get_timestamp(&self) -> u64 {
        (self.timestamp_hi as u64) << 32 | (self.timestamp_lo as u64)
    }

    pub fn get_profile_bytes(&self) -> [u8; BNO055_CALIB_SIZE] {
        let mut result = [0u8; BNO055_CALIB_SIZE];
        let profile_u8_slice =
            unsafe { core::slice::from_raw_parts(self.profile_data.as_ptr() as *const u8, 24) };
        result.copy_from_slice(&profile_u8_slice[..BNO055_CALIB_SIZE]);
        result
    }

    pub fn is_valid(&self) -> bool {
        self.magic == CALIBRATION_MAGIC
            && self.version == CALIBRATION_VERSION
            && self.checksum == self.calculate_checksum()
    }

    pub fn is_better_than(&self, current_quality: &CalibrationLevels) -> bool {
        let current_hash = CalibrationLevels::hash_quality(current_quality);
        self.quality_hash >= current_hash // Only replace if stored is same or better
    }

    fn calculate_checksum(&self) -> u32 {
        // XOR all u32 fields except checksum
        self.magic
            ^ self.version
            ^ self.timestamp_lo
            ^ self.timestamp_hi
            ^ self.quality_hash
            ^ self.profile_data.iter().fold(0u32, |acc, &x| acc ^ x)
    }

    pub fn as_u32_slice(&self) -> &[u32] {
        unsafe {
            core::slice::from_raw_parts(self as *const Self as *const u32, size_of::<Self>() / 4)
        }
    }
}
