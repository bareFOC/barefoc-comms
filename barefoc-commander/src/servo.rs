//! Smart servo extended ASCII commander.
//!
//! Provides ASCII console commands for runtime Control Table manipulation:
//! - `BOARD UID`: Print canonical 24-character uppercase hex hardware silicon UID
//! - `BOARD NAME [str]`: Query or configure board name (up to 15 chars)
//! - `BOARD ID [0-127]`: Query or configure board node ID
//! - `JOINT <idx> ADDR [0-127]`: Query or configure joint address (0-127)
//! - `JOINT <idx> NAME [str]`: Query or configure joint name
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
//! - `TM`: Query MOSFET stage temperature (legacy whole-degree channel)
//! - `TU`: Query MCU silicon temperature
//! - `TW`: Query motor winding temperature
//! - `LT <zone> <deg>`: Set a thermal limit (`F`/`TM` MOSFET, `U`/`TU` MCU,
//!   `M`/`W`/`TW` motor winding)
//! - `CUR` / `PWR` / `NRG`: Query bus current (A), bus power (W) and energy (W·h)
//! - `TRQ` / `ACC` / `MAG`: Query torque (N·m), acceleration (rad/s²) and field (mT)
//! - `WDT` / `CPU`: Query watchdog countdown (ms) and RTOS CPU load (%)
//! - `ODOM`: Query lifetime operating hours
//! - `CAP` / `ERRX`: Query capability bitmask and extended error flags as 8-digit hex
//!
//! All new queries preserve the legacy `TM`/`TU`/`TW`/`LT` grammar. Unequipped
//! floating-point channels report `NAN`.

use barefoc::control_table::{
    ControlTable, OperatingMode, TopologyPreset, REG_NODE_ID, REG_POS_P_GAIN, REG_VEL_P_GAIN,
};
use barefoc::traits::bootloader::BootloaderHandoff;
use barefoc::traits::storage::ConfigStorage;
use barefoc::traits::uid::{HardwareUid, MAX_UID_BYTES};

const MAX_NAME_LEN: usize = 15;

/// Smart servo ASCII command handler.
pub struct SmartServoCommander<'a, S = (), B = ()> {
    pub table: &'a mut ControlTable,
    pub storage: Option<&'a mut S>,
    pub bootloader: Option<&'a mut B>,
    pub align_handler: Option<fn(&mut ControlTable, usize)>,
    board_name: [u8; MAX_NAME_LEN + 1],
    board_id: u8,
    joint_addrs: [u8; 4],
    joint_names: [[u8; MAX_NAME_LEN + 1]; 4],
    uid_bytes: [u8; MAX_UID_BYTES],
    uid_len: usize,
    response_buf: [u8; 64],
}

impl<'a, S, B> SmartServoCommander<'a, S, B> {
    /// Create a new smart servo commander wrapping `table` and optional persistence/bootloader drivers.
    pub fn new(
        table: &'a mut ControlTable,
        storage: Option<&'a mut S>,
        bootloader: Option<&'a mut B>,
    ) -> Self {
        let initial_node_id = table.node_id();
        let mut s = Self {
            table,
            storage,
            bootloader,
            align_handler: None,
            board_name: [0u8; MAX_NAME_LEN + 1],
            board_id: initial_node_id,
            joint_addrs: [1, 2, 3, 4],
            joint_names: [[0u8; MAX_NAME_LEN + 1]; 4],
            uid_bytes: [0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF, 0x01, 0x23, 0x45, 0x67],
            uid_len: 12,
            response_buf: [0u8; 64],
        };
        s.set_board_name("barefoc_board");
        s.set_joint_name(0, "joint0");
        s.set_joint_name(1, "joint1");
        s.set_joint_name(2, "joint2");
        s.set_joint_name(3, "joint3");
        s
    }

    /// Set hardware UID bytes.
    pub fn with_uid_bytes(mut self, bytes: [u8; MAX_UID_BYTES], len: usize) -> Self {
        self.uid_bytes = bytes;
        self.uid_len = len.min(MAX_UID_BYTES);
        self
    }

    /// Set hardware UID from a provider implementing [`HardwareUid`].
    pub fn with_hardware_uid<U: HardwareUid>(mut self, uid: &U) -> Self {
        self.uid_bytes = uid.uid_bytes();
        self.uid_len = uid.uid_len().min(MAX_UID_BYTES);
        self
    }

    /// Set runtime calibration alignment handler.
    pub fn with_align_handler(mut self, handler: fn(&mut ControlTable, usize)) -> Self {
        self.align_handler = Some(handler);
        self
    }

    /// Get current board name.
    pub fn board_name(&self) -> &str {
        let len = self.board_name.iter().position(|&c| c == 0).unwrap_or(self.board_name.len());
        core::str::from_utf8(&self.board_name[..len]).unwrap_or("")
    }

    /// Set board name (truncated to 15 characters).
    pub fn set_board_name(&mut self, name: &str) {
        self.board_name.fill(0);
        let bytes = name.as_bytes();
        let copy_len = bytes.len().min(MAX_NAME_LEN);
        self.board_name[..copy_len].copy_from_slice(&bytes[..copy_len]);
    }

    /// Get board node ID.
    pub fn board_id(&self) -> u8 {
        self.board_id
    }

    /// Set board node ID.
    pub fn set_board_id(&mut self, id: u8) {
        self.board_id = id;
        self.table.write_u8(REG_NODE_ID, id);
    }

    /// Get joint address for index (0..3).
    pub fn joint_addr(&self, idx: usize) -> Option<u8> {
        if idx < 4 {
            Some(self.joint_addrs[idx])
        } else {
            None
        }
    }

    /// Set joint address for index (0..3).
    pub fn set_joint_addr(&mut self, idx: usize, addr: u8) -> bool {
        if idx < 4 {
            self.joint_addrs[idx] = addr;
            true
        } else {
            false
        }
    }

    /// Get joint name for index (0..3).
    pub fn joint_name(&self, idx: usize) -> Option<&str> {
        if idx < 4 {
            let name_buf = &self.joint_names[idx];
            let len = name_buf.iter().position(|&c| c == 0).unwrap_or(name_buf.len());
            core::str::from_utf8(&name_buf[..len]).ok()
        } else {
            None
        }
    }

    /// Set joint name for index (0..3).
    pub fn set_joint_name(&mut self, idx: usize, name: &str) -> bool {
        if idx < 4 {
            self.joint_names[idx].fill(0);
            let bytes = name.as_bytes();
            let copy_len = bytes.len().min(MAX_NAME_LEN);
            self.joint_names[idx][..copy_len].copy_from_slice(&bytes[..copy_len]);
            true
        } else {
            false
        }
    }

    fn format_uid_response(&mut self) -> &str {
        const HEX_CHARS: &[u8; 16] = b"0123456789ABCDEF";
        let len = self.uid_len.min(MAX_UID_BYTES);
        let out_chars = len * 2;
        for i in 0..len {
            let b = self.uid_bytes[i];
            self.response_buf[2 * i] = HEX_CHARS[(b >> 4) as usize];
            self.response_buf[2 * i + 1] = HEX_CHARS[(b & 0x0F) as usize];
        }
        self.response_buf[out_chars] = b'\n';
        core::str::from_utf8(&self.response_buf[..out_chars + 1]).unwrap_or("")
    }

    fn format_board_name_response(&mut self) -> &str {
        let len = self.board_name.iter().position(|&c| c == 0).unwrap_or(self.board_name.len());
        self.response_buf[..len].copy_from_slice(&self.board_name[..len]);
        self.response_buf[len] = b'\n';
        core::str::from_utf8(&self.response_buf[..len + 1]).unwrap_or("")
    }

    fn format_joint_name_response(&mut self, idx: usize) -> &str {
        let len = self.joint_names[idx].iter().position(|&c| c == 0).unwrap_or(self.joint_names[idx].len());
        self.response_buf[..len].copy_from_slice(&self.joint_names[idx][..len]);
        self.response_buf[len] = b'\n';
        core::str::from_utf8(&self.response_buf[..len + 1]).unwrap_or("")
    }

    fn format_u8_response(&mut self, val: u8) -> &str {
        let mut tmp = [0u8; 4];
        let len = if val >= 100 {
            tmp[0] = b'0' + (val / 100);
            tmp[1] = b'0' + ((val / 10) % 10);
            tmp[2] = b'0' + (val % 10);
            3
        } else if val >= 10 {
            tmp[0] = b'0' + (val / 10);
            tmp[1] = b'0' + (val % 10);
            2
        } else {
            tmp[0] = b'0' + val;
            1
        };
        self.response_buf[..len].copy_from_slice(&tmp[..len]);
        self.response_buf[len] = b'\n';
        core::str::from_utf8(&self.response_buf[..len + 1]).unwrap_or("")
    }

    /// Format a whole-degree temperature (legacy u16 channel) with a newline.
    fn format_u16_response(&mut self, val: u16) -> &str {
        let mut tmp = [0u8; 5];
        let mut i = 0;
        let mut v = val;
        if v == 0 {
            tmp[0] = b'0';
            i = 1;
        } else {
            while v > 0 {
                tmp[i] = b'0' + (v % 10) as u8;
                v /= 10;
                i += 1;
            }
        }
        let mut at = 0;
        while i > 0 {
            i -= 1;
            self.response_buf[at] = tmp[i];
            at += 1;
        }
        self.response_buf[at] = b'\n';
        core::str::from_utf8(&self.response_buf[..at + 1]).unwrap_or("")
    }

    /// Format a `u32` bitmask as eight uppercase hexadecimal digits plus newline.
    fn format_u32_hex_response(&mut self, val: u32) -> &str {
        const HEX_CHARS: &[u8; 16] = b"0123456789ABCDEF";
        for i in 0..8 {
            let shift = 28 - i * 4;
            self.response_buf[i] = HEX_CHARS[((val >> shift) & 0xF) as usize];
        }
        self.response_buf[8] = b'\n';
        core::str::from_utf8(&self.response_buf[..9]).unwrap_or("")
    }

    /// Format a temperature sample as a fixed two-decimal ASCII value.
    ///
    /// `f32::NAN` (unequipped zone) is reported as `NAN`. Uses `libm` for
    /// absolute value / rounding so the formatter stays `#![no_std]`.
    fn format_temp_response(&mut self, temp: f32) -> &str {
        if temp.is_nan() {
            self.response_buf[..4].copy_from_slice(b"NAN\n");
            return core::str::from_utf8(&self.response_buf[..4]).unwrap_or("NAN\n");
        }
        let negative = temp < 0.0;
        let scaled = libm::roundf(libm::fabsf(temp) * 100.0) as u32;
        let whole = scaled / 100;
        let frac = scaled % 100;
        let mut i = 0;
        if negative {
            self.response_buf[i] = b'-';
            i += 1;
        }
        let mut digits = [0u8; 10];
        let mut n = 0;
        let mut w = whole;
        if w == 0 {
            digits[0] = b'0';
            n = 1;
        } else {
            while w > 0 {
                digits[n] = b'0' + (w % 10) as u8;
                w /= 10;
                n += 1;
            }
        }
        while n > 0 {
            n -= 1;
            self.response_buf[i] = digits[n];
            i += 1;
        }
        self.response_buf[i] = b'.';
        i += 1;
        self.response_buf[i] = b'0' + (frac / 10) as u8;
        i += 1;
        self.response_buf[i] = b'0' + (frac % 10) as u8;
        i += 1;
        self.response_buf[i] = b'\n';
        core::str::from_utf8(&self.response_buf[..i + 1]).unwrap_or("")
    }
}

impl<'a, S: ConfigStorage, B: BootloaderHandoff> SmartServoCommander<'a, S, B> {
    /// Parse and execute an ASCII command string, returning a response string.
    pub fn parse_line(&mut self, line: &str) -> &str {
        let trimmed = line.trim();
        if trimmed.is_empty() {
            return "";
        }

        // BOARD commands: BOARD UID, BOARD NAME [str], BOARD ID [0-127]
        if trimmed.starts_with("BOARD") || trimmed.starts_with("board") {
            let rest = trimmed[5..].trim_start();
            let mut parts = rest.split_whitespace();
            if let Some(subcmd) = parts.next() {
                if subcmd.eq_ignore_ascii_case("UID") {
                    return self.format_uid_response();
                } else if subcmd.eq_ignore_ascii_case("NAME") {
                    if let Some(name_first) = parts.next() {
                        let offset = rest.find(name_first).unwrap_or(0);
                        let name_str = rest[offset..].trim();
                        self.set_board_name(name_str);
                        return "BOARD NAME SET\n";
                    } else {
                        return self.format_board_name_response();
                    }
                } else if subcmd.eq_ignore_ascii_case("ID") {
                    if let Some(id_str) = parts.next() {
                        if let Ok(id) = id_str.parse::<u8>() {
                            if id <= 127 {
                                self.set_board_id(id);
                                return "BOARD ID SET\n";
                            }
                        }
                        return "ERROR: INVALID ID\n";
                    } else {
                        let id = self.board_id();
                        return self.format_u8_response(id);
                    }
                }
            }
            return "ERROR: INVALID BOARD CMD\n";
        }

        // JOINT commands: JOINT <idx> ADDR [0-127], JOINT <idx> NAME [str]
        if trimmed.starts_with("JOINT") || trimmed.starts_with("joint") {
            let rest = trimmed[5..].trim_start();
            let mut parts = rest.split_whitespace();
            if let Some(idx_str) = parts.next() {
                if let Ok(idx) = idx_str.parse::<usize>() {
                    if idx >= 4 {
                        return "ERROR: INVALID JOINT\n";
                    }
                    if let Some(subcmd) = parts.next() {
                        if subcmd.eq_ignore_ascii_case("ADDR") {
                            if let Some(addr_str) = parts.next() {
                                if let Ok(addr) = addr_str.parse::<u8>() {
                                    if addr <= 127 {
                                        self.joint_addrs[idx] = addr;
                                        return "JOINT ADDR SET\n";
                                    }
                                }
                                return "ERROR: INVALID ADDR\n";
                            } else {
                                let addr = self.joint_addrs[idx];
                                return self.format_u8_response(addr);
                            }
                        } else if subcmd.eq_ignore_ascii_case("NAME") {
                            if let Some(name_first) = parts.next() {
                                let offset = rest.rfind(name_first).unwrap_or(0);
                                let name_str = rest[offset..].trim();
                                self.set_joint_name(idx, name_str);
                                return "JOINT NAME SET\n";
                            } else {
                                return self.format_joint_name_response(idx);
                            }
                        }
                    }
                }
            }
            return "ERROR: INVALID JOINT CMD\n";
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

        // Multi-zone thermal queries:
        //   TM -> MOSFET stage (legacy whole-degree channel at 0x142)
        //   TU -> MCU silicon (0x16A)
        //   TW -> motor windings (0x16E)
        // Unequipped zones report NAN.
        if trimmed.eq_ignore_ascii_case("TM") {
            let value = self.table.present_temp_mosfet() as u16;
            return self.format_u16_response(value);
        }
        if trimmed.eq_ignore_ascii_case("TU") {
            return self.format_temp_response(self.table.present_temp_mcu());
        }
        if trimmed.eq_ignore_ascii_case("TW") {
            return self.format_temp_response(self.table.present_temp_motor());
        }

        // Thermal limit configuration: LT <zone> <deg>.
        // Zone codes follow the query mnemonics; `F` is accepted as an alias
        // for the MOSFET/FET stage and `W` for the motor winding. The single
        // letter `M` maps to the motor zone (per the Commander spec scenario
        // `LT M 90` -> REG_TEMP_LIMIT_MOTOR).
        if trimmed.len() >= 2 && trimmed[..2].eq_ignore_ascii_case("LT") {
            let rest = trimmed[2..].trim_start();
            let mut parts = rest.split_whitespace();
            if let (Some(zone), Some(value_str)) = (parts.next(), parts.next()) {
                let limit = match crate::parse_f32(value_str) {
                    Ok(v) if v > 0.0 && v <= 200.0 => v,
                    _ => return "ERROR: INVALID LIMIT\n",
                };
                if zone.eq_ignore_ascii_case("F") || zone.eq_ignore_ascii_case("TM") {
                    self.table.set_temp_limit_mosfet(limit);
                } else if zone.eq_ignore_ascii_case("U") || zone.eq_ignore_ascii_case("TU") {
                    self.table.set_temp_limit_mcu(limit);
                } else if zone.eq_ignore_ascii_case("M")
                    || zone.eq_ignore_ascii_case("W")
                    || zone.eq_ignore_ascii_case("TW")
                {
                    self.table.set_temp_limit_motor(limit);
                } else {
                    return "ERROR: INVALID ZONE\n";
                }
                return "LIMIT SET\n";
            }
            return "ERROR: INVALID LT COMMAND\n";
        }

        // Extended telemetry queries (`extended-telemetry-controls`).
        // These are checked before the single-letter motor/P/V/G commands so
        // mnemonics such as `MAG` and `PWR` are not swallowed by that grammar.
        if trimmed.eq_ignore_ascii_case("CUR") {
            return self.format_temp_response(self.table.present_bus_current());
        }
        if trimmed.eq_ignore_ascii_case("PWR") {
            return self.format_temp_response(self.table.present_bus_power());
        }
        if trimmed.eq_ignore_ascii_case("NRG") {
            return self.format_temp_response(self.table.accumulated_energy());
        }
        if trimmed.eq_ignore_ascii_case("TRQ") {
            return self.format_temp_response(self.table.present_torque());
        }
        if trimmed.eq_ignore_ascii_case("ACC") {
            return self.format_temp_response(self.table.present_acceleration());
        }
        if trimmed.eq_ignore_ascii_case("MAG") {
            return self.format_temp_response(self.table.mag_field_magnitude());
        }
        if trimmed.eq_ignore_ascii_case("ODOM") {
            return self.format_temp_response(self.table.total_operating_hours());
        }
        if trimmed.eq_ignore_ascii_case("WDT") {
            let remaining = self.table.watchdog_time_left_ms();
            return self.format_u16_response(remaining);
        }
        if trimmed.eq_ignore_ascii_case("CPU") {
            return self.format_u8_response(self.table.cpu_load_pct());
        }
        if trimmed.eq_ignore_ascii_case("CAP") {
            return self.format_u32_hex_response(self.table.hardware_capabilities());
        }
        if trimmed.eq_ignore_ascii_case("ERRX") {
            return self.format_u32_hex_response(self.table.hardware_error_status_ext());
        }

        if trimmed.eq_ignore_ascii_case("ALIGN") {
            if let Some(handler) = self.align_handler {
                handler(self.table, 0);
            }
            if let Some(ref mut storage) = self.storage {
                let _ = storage.save_config(self.table);
            }
            return "ALIGN OK\n";
        }

        if trimmed.eq_ignore_ascii_case("ZERO") {
            let pos = self.table.present_position();
            self.table.set_homing_offset(pos);
            self.table.set_motor_homing_offset(0, pos);
            if let Some(ref mut storage) = self.storage {
                let _ = storage.save_config(self.table);
            }
            return "ZERO OK\n";
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
                        } else if subcmd.eq_ignore_ascii_case("ALIGN") {
                            let axis = _id as usize;
                            if axis < 4 {
                                if let Some(handler) = self.align_handler {
                                    handler(self.table, axis);
                                }
                                if let Some(ref mut storage) = self.storage {
                                    let _ = storage.save_config(self.table);
                                }
                                return "ALIGN OK\n";
                            }
                            return "ERROR: INVALID JOINT\n";
                        } else if subcmd.eq_ignore_ascii_case("ZERO") {
                            let axis = _id as usize;
                            if axis < 4 {
                                let pos = self.table.present_position();
                                self.table.set_homing_offset(pos);
                                self.table.set_motor_homing_offset(axis, pos);
                                if let Some(ref mut storage) = self.storage {
                                    let _ = storage.save_config(self.table);
                                }
                                return "ZERO OK\n";
                            }
                            return "ERROR: INVALID JOINT\n";
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
    fn test_identity_commands() {
        let mut table = ControlTable::new();
        let mut storage = MockStorage { save_count: 0 };
        let mut boot = MockBootloader { entered: false };
        let mut cmd = SmartServoCommander::new(&mut table, Some(&mut storage), Some(&mut boot));

        // Test BOARD UID
        let uid_resp = cmd.parse_line("BOARD UID");
        assert_eq!(uid_resp, "0123456789ABCDEF01234567\n");

        // Test BOARD NAME query & set
        assert_eq!(cmd.parse_line("BOARD NAME"), "barefoc_board\n");
        assert_eq!(cmd.parse_line("BOARD NAME arm_wrist"), "BOARD NAME SET\n");
        assert_eq!(cmd.parse_line("BOARD NAME"), "arm_wrist\n");
        assert_eq!(cmd.board_name(), "arm_wrist");

        // Test BOARD ID query & set
        assert_eq!(cmd.parse_line("BOARD ID"), "64\n");
        assert_eq!(cmd.parse_line("BOARD ID 42"), "BOARD ID SET\n");
        assert_eq!(cmd.parse_line("BOARD ID"), "42\n");
        assert_eq!(cmd.board_id(), 42);
        assert_eq!(cmd.table.node_id(), 42);
        assert_eq!(cmd.parse_line("BOARD ID 200"), "ERROR: INVALID ID\n");

        // Test JOINT ADDR query & set
        assert_eq!(cmd.parse_line("JOINT 0 ADDR"), "1\n");
        assert_eq!(cmd.parse_line("JOINT 0 ADDR 5"), "JOINT ADDR SET\n");
        assert_eq!(cmd.parse_line("JOINT 0 ADDR"), "5\n");
        assert_eq!(cmd.joint_addr(0), Some(5));
        assert_eq!(cmd.parse_line("JOINT 4 ADDR 5"), "ERROR: INVALID JOINT\n");

        // Test JOINT NAME query & set
        assert_eq!(cmd.parse_line("JOINT 1 NAME"), "joint1\n");
        assert_eq!(cmd.parse_line("JOINT 1 NAME wrist_roll"), "JOINT NAME SET\n");
        assert_eq!(cmd.parse_line("JOINT 1 NAME"), "wrist_roll\n");
        assert_eq!(cmd.joint_name(1), Some("wrist_roll"));
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

    fn test_align_handler(table: &mut ControlTable, axis: usize) {
        table.set_motor_zero_electric_angle(axis, 3.14);
        table.set_motor_sensor_direction(axis, 1);
    }

    #[test]
    fn test_align_and_zero_commands() {
        let mut table = ControlTable::new();
        table.set_present_position(2.718);
        let mut storage = MockStorage { save_count: 0 };
        let mut boot = MockBootloader { entered: false };
        let mut cmd = SmartServoCommander::new(&mut table, Some(&mut storage), Some(&mut boot))
            .with_align_handler(test_align_handler);

        // ALIGN top-level
        let resp1 = cmd.parse_line("ALIGN");
        assert_eq!(resp1, "ALIGN OK\n");
        assert_eq!(cmd.table.motor_zero_electric_angle(0), 3.14);
        assert_eq!(cmd.table.motor_sensor_direction(0), 1);
        assert_eq!(cmd.storage.as_ref().unwrap().save_count, 1);

        // M1 ALIGN
        let resp2 = cmd.parse_line("M1 ALIGN");
        assert_eq!(resp2, "ALIGN OK\n");
        assert_eq!(cmd.table.motor_zero_electric_angle(1), 3.14);
        assert_eq!(cmd.table.motor_sensor_direction(1), 1);
        assert_eq!(cmd.storage.as_ref().unwrap().save_count, 2);

        // ZERO top-level
        let resp3 = cmd.parse_line("ZERO");
        assert_eq!(resp3, "ZERO OK\n");
        assert_eq!(cmd.table.homing_offset(), 2.718);
        assert_eq!(cmd.table.motor_homing_offset(0), 2.718);
        assert_eq!(cmd.storage.as_ref().unwrap().save_count, 3);

        // M2 ZERO
        cmd.table.set_present_position(0.5);
        let resp4 = cmd.parse_line("M2 ZERO");
        assert_eq!(resp4, "ZERO OK\n");
        assert_eq!(cmd.table.motor_homing_offset(2), 0.5);
        assert_eq!(cmd.storage.as_ref().unwrap().save_count, 4);
    }

    #[test]
    fn test_thermal_query_commands() {
        let mut table = ControlTable::new();
        table.set_present_temp_mosfet(55.9);
        table.set_present_temp_mcu(61.25);
        table.set_present_temp_motor(72.5);
        let mut storage = MockStorage { save_count: 0 };
        let mut boot = MockBootloader { entered: false };
        let mut cmd = SmartServoCommander::new(&mut table, Some(&mut storage), Some(&mut boot));

        // TM reports the legacy whole-degree channel without decimals.
        assert_eq!(cmd.parse_line("TM"), "55\n");
        assert_eq!(cmd.parse_line("tu"), "61.25\n");
        assert_eq!(cmd.parse_line("TW"), "72.50\n");

        // Unequipped zones report NAN.
        cmd.table.set_present_temp_motor(f32::NAN);
        assert_eq!(cmd.parse_line("TW"), "NAN\n");
    }

    #[test]
    fn test_thermal_limit_command_and_validation() {
        let mut table = ControlTable::new();
        let mut storage = MockStorage { save_count: 0 };
        let mut boot = MockBootloader { entered: false };
        let mut cmd = SmartServoCommander::new(&mut table, Some(&mut storage), Some(&mut boot));

        // Spec scenario: LT M 90 targets the motor-winding limit.
        assert_eq!(cmd.parse_line("LT M 90"), "LIMIT SET\n");
        assert_eq!(cmd.table.temp_limit_motor(), 90.0);
        assert_eq!(cmd.table.temp_limit_mosfet(), 75.0);

        // All zone codes, including the TM/TU/TW mnemonics and lowercase input.
        assert_eq!(cmd.parse_line("LT F 80.5"), "LIMIT SET\n");
        assert_eq!(cmd.table.temp_limit_mosfet(), 80.5);
        assert_eq!(cmd.parse_line("LT TM 81"), "LIMIT SET\n");
        assert_eq!(cmd.table.temp_limit_mosfet(), 81.0);
        assert_eq!(cmd.parse_line("LT U 85.25"), "LIMIT SET\n");
        assert_eq!(cmd.table.temp_limit_mcu(), 85.25);
        assert_eq!(cmd.parse_line("LT tu 86"), "LIMIT SET\n");
        assert_eq!(cmd.table.temp_limit_mcu(), 86.0);
        assert_eq!(cmd.parse_line("LT W 91"), "LIMIT SET\n");
        assert_eq!(cmd.table.temp_limit_motor(), 91.0);
        assert_eq!(cmd.parse_line("LT TW 92"), "LIMIT SET\n");
        assert_eq!(cmd.table.temp_limit_motor(), 92.0);

        // Validation: unknown zone, non-numeric, zero, negative and out-of-range.
        assert_eq!(cmd.parse_line("LT X 90"), "ERROR: INVALID ZONE\n");
        assert_eq!(cmd.parse_line("LT M abc"), "ERROR: INVALID LIMIT\n");
        assert_eq!(cmd.parse_line("LT M 0"), "ERROR: INVALID LIMIT\n");
        assert_eq!(cmd.parse_line("LT M -5"), "ERROR: INVALID LIMIT\n");
        assert_eq!(cmd.parse_line("LT M 250"), "ERROR: INVALID LIMIT\n");
        assert_eq!(cmd.parse_line("LT M"), "ERROR: INVALID LT COMMAND\n");
        assert_eq!(cmd.parse_line("LT"), "ERROR: INVALID LT COMMAND\n");
        assert_eq!(cmd.table.temp_limit_motor(), 92.0);
    }

    #[test]
    fn test_extended_telemetry_query_commands() {
        let mut table = ControlTable::new();
        table.set_present_bus_current(1.25);
        table.set_present_bus_power(7.5);
        table.set_accumulated_energy(3.5);
        table.set_present_torque(0.4);
        table.set_present_acceleration(12.5);
        table.set_mag_field_magnitude(42.5);
        table.set_total_operating_hours(125.75);
        table.set_watchdog_timeout_ms(500);
        table.feed_watchdog();
        table.set_cpu_load_pct(37);
        table.set_hardware_capabilities(
            barefoc::control_table::capabilities::DUAL_ENCODER
                | barefoc::control_table::capabilities::POWER_TELEMETRY,
        );
        table.set_error_flag_ext(barefoc::control_table::error_flags_ext::STALL_DETECTED);

        let mut storage = MockStorage { save_count: 0 };
        let mut boot = MockBootloader { entered: false };
        let mut cmd = SmartServoCommander::new(&mut table, Some(&mut storage), Some(&mut boot));

        assert_eq!(cmd.parse_line("CUR"), "1.25\n");
        assert_eq!(cmd.parse_line("PWR"), "7.50\n");
        assert_eq!(cmd.parse_line("NRG"), "3.50\n");
        assert_eq!(cmd.parse_line("TRQ"), "0.40\n");
        assert_eq!(cmd.parse_line("ACC"), "12.50\n");
        assert_eq!(cmd.parse_line("MAG"), "42.50\n");
        assert_eq!(cmd.parse_line("ODOM"), "125.75\n");
        assert_eq!(cmd.parse_line("WDT"), "500\n");
        assert_eq!(cmd.parse_line("CPU"), "37\n");
        assert_eq!(cmd.parse_line("cap"), "00000022\n");
        assert_eq!(cmd.parse_line("ERRX"), "00010000\n");
    }

    #[test]
    fn test_extended_queries_do_not_break_legacy_commands() {
        let mut table = ControlTable::new();
        table.set_present_temp_mosfet(55.0);
        table.set_present_temp_mcu(61.25);
        table.set_present_temp_motor(72.5);
        let mut storage = MockStorage { save_count: 0 };
        let mut boot = MockBootloader { entered: false };
        let mut cmd = SmartServoCommander::new(&mut table, Some(&mut storage), Some(&mut boot));

        // Legacy thermal queries are unchanged.
        assert_eq!(cmd.parse_line("TM"), "55\n");
        assert_eq!(cmd.parse_line("TU"), "61.25\n");
        assert_eq!(cmd.parse_line("TW"), "72.50\n");
        assert_eq!(cmd.parse_line("LT M 80"), "LIMIT SET\n");
        assert_eq!(cmd.table.temp_limit_motor(), 80.0);

        // New queries coexist with them, including the NAN sentinel.
        assert_eq!(cmd.parse_line("PWR"), "NAN\n");
        assert_eq!(cmd.parse_line("CAP"), "00000000\n");
        // Single-letter motor grammar still dispatches correctly.
        assert_eq!(cmd.parse_line("M0 C 3"), "MODE SET\n");
        assert_eq!(cmd.parse_line("P 35.5"), "P GAIN SET\n");
        assert_eq!(cmd.parse_line("V 0.5"), "V GAIN SET\n");
    }
}
