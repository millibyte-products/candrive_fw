//! Software CRC-32/MPEG-2 (poly 0x04C11DB7, init 0xFFFFFFFF, no reflection,
//! no XOR-out). Matches the convention of the STM32F1 hardware CRC unit so
//! firmware can verify offline-computed checksums regardless of whether the
//! HW CRC is currently clocked.

/// Streaming CRC-32/MPEG-2 state.
///
/// The host-side `tools/fw_update.py` and the bootloader both feed bytes
/// through this so a freshly streamed image can be validated end-to-end
/// without ever needing to read flash back.
#[derive(Clone, Copy, Debug)]
pub struct Crc32Mpeg2 {
    state: u32,
}

impl Crc32Mpeg2 {
    pub const INIT: u32 = 0xFFFF_FFFF;
    pub const POLY: u32 = 0x04C1_1DB7;

    #[inline]
    pub const fn new() -> Self { Self { state: Self::INIT } }

    pub fn update(&mut self, bytes: &[u8]) {
        let mut crc = self.state;
        for &b in bytes {
            crc ^= (b as u32) << 24;
            // Unrolled 8-bit step.
            for _ in 0..8 {
                crc = if crc & 0x8000_0000 != 0 {
                    (crc << 1) ^ Self::POLY
                } else {
                    crc << 1
                };
            }
        }
        self.state = crc;
    }

    #[inline]
    pub const fn finalize(self) -> u32 { self.state }
}

impl Default for Crc32Mpeg2 {
    fn default() -> Self { Self::new() }
}

/// Convenience: compute the CRC of an entire buffer in one shot.
pub fn crc32_mpeg2(bytes: &[u8]) -> u32 {
    let mut c = Crc32Mpeg2::new();
    c.update(bytes);
    c.finalize()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_is_init() {
        assert_eq!(crc32_mpeg2(&[]), Crc32Mpeg2::INIT);
    }

    #[test]
    fn known_vector_123456789() {
        // Standard CRC-32/MPEG-2 check vector for ASCII "123456789".
        assert_eq!(crc32_mpeg2(b"123456789"), 0x0376_E6E7);
    }

    #[test]
    fn streaming_matches_oneshot() {
        let data: &[u8] = b"the quick brown fox jumps over the lazy dog";
        let one = crc32_mpeg2(data);
        let mut c = Crc32Mpeg2::new();
        for chunk in data.chunks(7) {
            c.update(chunk);
        }
        assert_eq!(c.finalize(), one);
    }
}
