//! In-band bootloader strobe decoder and ISR-level execution hooks.
//!
//! Provides deterministic detection and ISR-level preemption for in-band bootloader
//! entry commands across address-based bus transports (I2C, I3C, and MultiBus):
//! - Single-byte `0xF0` (`I2C_CMD_ENTER_BOOTLOADER`)
//! - MultiBus 8-bit pointer write of `0xF0` to `RebootCommand` (`[0x63, 0xF0]`)
//! - MultiBus 16-bit pointer write of `0xF0` to `RebootCommand` (`[0x00, 0x63, 0xF0]`)
//! - Universal ControlTable 16-bit pointer write of `0xF0` to `0x0123` (`[0x01, 0x23, 0xF0]`)
//! - I3C CCC `RSTACT` target reset (`0x02`) or raw `0xF0` strobe

pub const I2C_CMD_ENTER_BOOTLOADER: u8 = 0xF0;
pub const I3C_CCC_RSTACT_BCAST: u8 = 0x2A;
pub const I3C_CCC_RSTACT_DIRECT: u8 = 0x9A;
pub const I3C_RSTACT_TARGET_RESET: u8 = 0x02;

/// Pure function to evaluate whether an incoming raw bus frame is an in-band
/// bootloader entry strobe.
///
/// Returns `true` if:
/// 1. Exactly one byte `[0xF0]`
/// 2. MultiBus 8-bit pointer write: `[0x63, 0xF0]`
/// 3. MultiBus 16-bit pointer write: `[0x00, 0x63, 0xF0]`
/// 4. ControlTable 16-bit pointer write: `[0x01, 0x23, 0xF0]`
pub fn is_bootloader_strobe(frame: &[u8]) -> bool {
    if frame.len() == 1 && frame[0] == I2C_CMD_ENTER_BOOTLOADER {
        return true;
    }
    if frame.len() >= 2 && frame[0] == 0x63 && frame[1] == I2C_CMD_ENTER_BOOTLOADER {
        return true;
    }
    if frame.len() >= 3 && frame[0] == 0x00 && frame[1] == 0x63 && frame[2] == I2C_CMD_ENTER_BOOTLOADER {
        return true;
    }
    if frame.len() >= 3 && frame[0] == 0x01 && frame[1] == 0x23 && frame[2] == I2C_CMD_ENTER_BOOTLOADER {
        return true;
    }
    false
}

/// Decode an I3C CCC `RSTACT` target reset strobe.
pub fn is_i3c_bootloader_strobe(ccc: u8, defining: u8) -> bool {
    (ccc == I3C_CCC_RSTACT_BCAST || ccc == I3C_CCC_RSTACT_DIRECT)
        && defining == I3C_RSTACT_TARGET_RESET
}

/// Execute bootloader entry if the received frame matches a bootloader strobe.
///
/// Designed to be called directly inside an interrupt service routine (ISR)
/// or high-priority interrupt executor task to preempt starved application tasks.
#[cfg(feature = "control-table")]
pub fn multibus_isr_strobe_hook<B: barefoc::traits::bootloader::BootloaderHandoff>(
    frame: &[u8],
    bootloader: &mut B,
) -> bool {
    if is_bootloader_strobe(frame) {
        bootloader.enter_bootloader();
    }
    false
}

#[cfg(test)]
mod tests {
    use super::*;

    struct MockBootloader {
        entered: bool,
    }

    impl MockBootloader {
        fn new() -> Self {
            Self { entered: false }
        }
    }

    #[cfg(feature = "control-table")]
    impl barefoc::traits::bootloader::BootloaderHandoff for MockBootloader {
        fn enter_bootloader(&mut self) -> ! {
            self.entered = true;
            panic!("MOCK_BOOTLOADER_ENTERED");
        }
    }

    #[test]
    fn test_is_bootloader_strobe_positive_cases() {
        assert!(is_bootloader_strobe(&[0xF0]));
        assert!(is_bootloader_strobe(&[0x63, 0xF0]));
        assert!(is_bootloader_strobe(&[0x63, 0xF0, 0x00]));
        assert!(is_bootloader_strobe(&[0x00, 0x63, 0xF0]));
        assert!(is_bootloader_strobe(&[0x01, 0x23, 0xF0]));
    }

    #[test]
    fn test_is_bootloader_strobe_negative_cases() {
        assert!(!is_bootloader_strobe(&[]));
        assert!(!is_bootloader_strobe(&[0x00]));
        assert!(!is_bootloader_strobe(&[0xF0, 0x00]));
        assert!(!is_bootloader_strobe(&[0x63, 0x00]));
        assert!(!is_bootloader_strobe(&[0x00, 0x63, 0x00]));
        assert!(!is_bootloader_strobe(&[0x01, 0x23, 0x00]));
        assert!(!is_bootloader_strobe(&[0x00, 0x08, 0x11, 0x22]));
    }

    #[test]
    fn test_i3c_strobe_decoding() {
        assert!(is_i3c_bootloader_strobe(0x2A, 0x02));
        assert!(is_i3c_bootloader_strobe(0x9A, 0x02));
        assert!(!is_i3c_bootloader_strobe(0x2A, 0x00));
        assert!(!is_i3c_bootloader_strobe(0x10, 0x02));
    }

    #[test]
    #[cfg(feature = "control-table")]
    fn test_isr_strobe_preempts_stalled_task() {
        use std::sync::atomic::{AtomicBool, Ordering};
        use std::sync::Arc;
        use std::thread;

        let running = Arc::new(AtomicBool::new(true));
        let running_clone = running.clone();

        // Simulate a stalled background motor/cooperative task in another thread
        let handle = thread::spawn(move || {
            let mut counter: u64 = 0;
            while running_clone.load(Ordering::Relaxed) {
                counter = counter.wrapping_add(1);
                std::hint::spin_loop();
            }
            counter
        });

        // Simulate high-priority ISR receiving 0xF0 strobe
        let mut bl = MockBootloader::new();
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            multibus_isr_strobe_hook(&[0xF0], &mut bl);
        }));

        assert!(result.is_err());
        running.store(false, Ordering::Relaxed);
        handle.join().unwrap();
    }
}
