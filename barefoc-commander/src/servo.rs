//! Smart servo extended ASCII commander.
//!
//! Provides ASCII console commands for runtime Control Table manipulation:
//! - `M<id> C <mode>`: Set Operating Mode (0..7)
//! - `M<id> T <val>`: Set Goal Setpoint
//! - `K M <preset>`: Set Kinematics Topology Preset (0..3, 0xFF)
//! - `K T <joint> <val>`: Set Kinematic Joint Goal Setpoint
//! - `SAVE` or `S`: Commit EEPROM configuration to non-volatile flash
//! - `RELOAD` or `L`: Reload parameters from non-volatile flash
//! - `FACTORY`: Restore hardcoded factory defaults
//! - `!BOOT` or `B`: Disarm motor outputs and reboot into BareBoot Katapult
//! - `DIAG` or `D`: Print active hardware error bitfield and status
//! - `P <val>`: Set position loop proportional gain Kp
//! - `V <val>`: Set velocity loop proportional gain Kp

use barefoc::control_table::{ControlTable, OperatingMode, TopologyPreset, REG_POS_P_GAIN, REG_VEL_P_GAIN};
use barefoc::traits::bootloader::BootloaderHandoff;
use barefoc::traits::storage::ConfigStorage;

/// Smart servo ASCII command handler.
pub struct SmartServoCommander<'a, S = (), B = ()> {
    pub table: &'a mut ControlTable,
    pub storage: Option<&'a mut S>,
    pub bootloader: Option<&'a mut B>,
}

impl<'a, S, B> SmartServoCommander<'a, S, B> {
    /// Create a new smart servo commander wrapping `table` and optional persistence/bootloader drivers.
    pub fn new(
        table: &'a mut ControlTable,
        storage: Option<&'a mut S>,
        bootloader: Option<&'a mut B>,
    ) -> Self {
        Self {
            table,
            storage,
            bootloader,
        }
    }
}

impl<'a, S: ConfigStorage, B: BootloaderHandoff> SmartServoCommander<'a, S, B> {
    /// Parse and execute an ASCII command string, returning a static response label.
    pub fn parse_line(&mut self, line: &str) -> &'static str {
        let trimmed = line.trim();
        if trimmed.is_empty() {
            return "";
        }

        // Global persistence & lifecycle commands
        if trimmed.eq_ignore_ascii_case("SAVE") || trimmed == "S" {
            if let Some(ref mut storage) = self.storage {
                if storage.save_config(self.table).is_ok() {
                    return "CONFIG SAVED\n";
                } else {
                    return "ERROR: SAVE FAILED\n";
                }
            }
            return "ERROR: NO STORAGE\n";
        }

        if trimmed.eq_ignore_ascii_case("RELOAD") || trimmed == "L" {
            if let Some(ref mut storage) = self.storage {
                if storage.load_config(self.table).is_ok() {
                    return "CONFIG RELOADED\n";
                } else {
                    return "ERROR: RELOAD FAILED\n";
                }
            }
            return "ERROR: NO STORAGE\n";
        }

        if trimmed.eq_ignore_ascii_case("FACTORY") {
            if let Some(ref mut storage) = self.storage {
                if storage.factory_reset(self.table).is_ok() {
                    return "FACTORY RESET\n";
                } else {
                    return "ERROR: RESET FAILED\n";
                }
            }
            self.table.load_factory_defaults();
            return "FACTORY RESET\n";
        }

        if trimmed == "!BOOT" || trimmed == "B" {
            if let Some(ref mut bootloader) = self.bootloader {
                bootloader.enter_bootloader();
            }
            return "ERROR: NO BOOTLOADER\n";
        }

        if trimmed.eq_ignore_ascii_case("DIAG") || trimmed == "D" {
            let err = self.table.hardware_error_status();
            if err == 0 {
                return "STATUS OK\n";
            } else {
                return "HARDWARE ERROR DETECTED\n";
            }
        }

        // Motor commands: M<id> C <mode> or M<id> T <val>
        if trimmed.starts_with('M') || trimmed.starts_with('m') {
            let rest = trimmed[1..].trim_start();
            let mut parts = rest.split_whitespace();
            if let Some(id_str) = parts.next() {
                if let Ok(_id) = id_str.parse::<u8>() {
                    if let Some(subcmd) = parts.next() {
                        if subcmd.eq_ignore_ascii_case("C") {
                            if let Some(mode_str) = parts.next() {
                                if let Ok(m) = mode_str.parse::<u8>() {
                                    if let Ok(mode) = OperatingMode::try_from(m) {
                                        self.table.set_operating_mode(mode);
                                        return "MODE SET\n";
                                    }
                                }
                            }
                            return "ERROR: INVALID MODE\n";
                        } else if subcmd.eq_ignore_ascii_case("T") {
                            if let Some(val_str) = parts.next() {
                                if let Ok(v) = crate::parse_f32(val_str) {
                                    self.table.set_goal_position(v);
                                    self.table.set_goal_velocity(v);
                                    return "SETPOINT SET\n";
                                }
                            }
                            return "ERROR: INVALID SETPOINT\n";
                        }
                    }
                }
            }
        }

        // Kinematics commands: K M <preset> or K T <joint> <val>
        if trimmed.starts_with('K') || trimmed.starts_with('k') {
            let rest = trimmed[1..].trim_start();
            let mut parts = rest.split_whitespace();
            if let Some(subcmd) = parts.next() {
                if subcmd.eq_ignore_ascii_case("M") {
                    if let Some(preset_str) = parts.next() {
                        if let Ok(p) = preset_str.parse::<u8>() {
                            if let Ok(preset) = TopologyPreset::try_from(p) {
                                self.table.set_topology_preset(preset);
                                return "TOPOLOGY SET\n";
                            }
                        }
                    }
                    return "ERROR: INVALID TOPOLOGY\n";
                } else if subcmd.eq_ignore_ascii_case("T") {
                    if let (Some(_joint_str), Some(val_str)) = (parts.next(), parts.next()) {
                        if let Ok(v) = crate::parse_f32(val_str) {
                            self.table.set_goal_position(v);
                            return "JOINT SET\n";
                        }
                    }
                    return "ERROR: INVALID JOINT TARGET\n";
                }
            }
        }

        // Tuning commands: P <val> or V <val>
        if trimmed.starts_with('P') || trimmed.starts_with('p') {
            let rest = trimmed[1..].trim_start();
            if let Ok(p_gain) = crate::parse_f32(rest) {
                self.table.write_f32(REG_POS_P_GAIN, p_gain);
                return "P GAIN SET\n";
            }
        }

        if trimmed.starts_with('V') || trimmed.starts_with('v') {
            let rest = trimmed[1..].trim_start();
            if let Ok(v_gain) = crate::parse_f32(rest) {
                self.table.write_f32(REG_VEL_P_GAIN, v_gain);
                return "V GAIN SET\n";
            }
        }

        "ERROR: UNKNOWN COMMAND\n"
    }

    #[cfg(feature = "embedded-io")]
    pub fn execute_stream<W: embedded_io::Write>(
        &mut self,
        line: &str,
        writer: &mut W,
    ) -> Result<(), W::Error> {
        let resp = self.parse_line(line);
        if !resp.is_empty() {
            writer.write_all(resp.as_bytes())?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct MockStorage {
        save_count: usize,
    }

    impl ConfigStorage for MockStorage {
        type Error = ();

        fn load_config(&mut self, _table: &mut ControlTable) -> Result<(), Self::Error> {
            Ok(())
        }

        fn save_config(&mut self, _table: &ControlTable) -> Result<(), Self::Error> {
            self.save_count += 1;
            Ok(())
        }

        fn factory_reset(&mut self, _table: &mut ControlTable) -> Result<(), Self::Error> {
            Ok(())
        }
    }

    struct MockBootloader {
        entered: bool,
    }

    impl BootloaderHandoff for MockBootloader {
        fn enter_bootloader(&mut self) -> ! {
            self.entered = true;
            panic!("ENTER_BOOTLOADER");
        }
    }

    #[test]
    fn test_mode_and_target_commands() {
        let mut table = ControlTable::new();
        let mut storage = MockStorage { save_count: 0 };
        let mut boot = MockBootloader { entered: false };
        let mut cmd = SmartServoCommander::new(&mut table, Some(&mut storage), Some(&mut boot));

        let resp1 = cmd.parse_line("M0 C 3");
        assert_eq!(resp1, "MODE SET\n");
        assert_eq!(cmd.table.operating_mode(), OperatingMode::MultiTurnPosition);

        let resp2 = cmd.parse_line("M0 T 1.5707");
        assert_eq!(resp2, "SETPOINT SET\n");
        assert!((cmd.table.goal_position() - 1.5707).abs() < 1e-4);
    }

    #[test]
    fn test_kinematics_and_tuning_commands() {
        let mut table = ControlTable::new();
        let mut storage = MockStorage { save_count: 0 };
        let mut boot = MockBootloader { entered: false };
        let mut cmd = SmartServoCommander::new(&mut table, Some(&mut storage), Some(&mut boot));

        let resp1 = cmd.parse_line("K M 1");
        assert_eq!(resp1, "TOPOLOGY SET\n");
        assert_eq!(cmd.table.topology_preset(), TopologyPreset::Differential2x2Dof);

        let resp2 = cmd.parse_line("P 35.5");
        assert_eq!(resp2, "P GAIN SET\n");
        assert_eq!(cmd.table.read_f32(REG_POS_P_GAIN), 35.5);
    }

    #[test]
    fn test_persistence_and_status_commands() {
        let mut table = ControlTable::new();
        let mut storage = MockStorage { save_count: 0 };
        let mut boot = MockBootloader { entered: false };
        let mut cmd = SmartServoCommander::new(&mut table, Some(&mut storage), Some(&mut boot));

        let resp1 = cmd.parse_line("SAVE");
        assert_eq!(resp1, "CONFIG SAVED\n");
        assert_eq!(cmd.storage.as_ref().unwrap().save_count, 1);

        let resp2 = cmd.parse_line("DIAG");
        assert_eq!(resp2, "STATUS OK\n");
    }
}
