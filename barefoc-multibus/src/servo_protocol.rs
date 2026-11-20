//! Streaming serial servo frame parser and atomic SYNC_WRITE processor.
//!
//! Supports streaming frame decoding for Dynamixel Protocol 1.0, Feetech STS,
//! and Dynamixel Protocol 2.0 without dynamic memory allocation.

use crate::multi_id::{MultiIdRouter, RouteTarget};

/// Supported servo protocol variants.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ServoProtocolKind {
    /// Dynamixel v1 / Feetech STS protocol (0xFF 0xFF header, 8-bit checksum).
    V1,
    /// Dynamixel v2 protocol (0xFF 0xFF 0xFD 0x00 header, 16-bit CRC).
    V2,
}

/// Standard instructions.
pub const INST_PING: u8 = 0x01;
pub const INST_READ: u8 = 0x02;
pub const INST_WRITE: u8 = 0x03;
pub const INST_REG_WRITE: u8 = 0x04;
pub const INST_ACTION: u8 = 0x05;
pub const INST_FACTORY_RESET: u8 = 0x06;
pub const INST_REBOOT: u8 = 0x08;
pub const INST_SYNC_READ: u8 = 0x82;
pub const INST_SYNC_WRITE: u8 = 0x83;

/// Decoded servo packet with static parameter storage.
#[derive(Debug, Clone, PartialEq)]
pub struct ServoPacket<const MAX_PARAMS: usize = 128> {
    pub protocol: ServoProtocolKind,
    pub id: u8,
    pub instruction: u8,
    pub params: [u8; MAX_PARAMS],
    pub params_len: usize,
}

impl<const MAX_PARAMS: usize> ServoPacket<MAX_PARAMS> {
    /// Get slice of received parameter bytes.
    pub fn params(&self) -> &[u8] {
        &self.params[..self.params_len]
    }
}

/// Computes CRC-16 for Dynamixel v2 packets.
pub fn calc_dxl2_crc(bytes: &[u8]) -> u16 {
    let mut crc: u16 = 0;
    for &b in bytes {
        crc ^= (b as u16) << 8;
        for _ in 0..8 {
            if (crc & 0x8000) != 0 {
                crc = (crc << 1) ^ 0x8005;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ParserState {
    WaitHeader1,
    WaitHeader2,
    CheckVersionOrId,
    // V1 state
    V1Length,
    V1Instruction,
    V1Params,
    V1Checksum,
    // V2 state
    V2HeaderReserved,
    V2Id,
    V2LengthL,
    V2LengthH,
    V2Instruction,
    V2Params,
    V2CrcL,
    V2CrcH,
}

/// Streaming zero-allocation parser for serial servo frames.
pub struct StreamingServoParser<const MAX_PARAMS: usize = 128> {
    state: ParserState,
    id: u8,
    instruction: u8,
    length: usize,
    index: usize,
    checksum_acc: u8,
    crc_acc: u16,
    crc_expected: u16,
    params: [u8; MAX_PARAMS],
}

impl<const MAX_PARAMS: usize> Default for StreamingServoParser<MAX_PARAMS> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const MAX_PARAMS: usize> StreamingServoParser<MAX_PARAMS> {
    /// Create a new streaming parser.
    pub const fn new() -> Self {
        Self {
            state: ParserState::WaitHeader1,
            id: 0,
            instruction: 0,
            length: 0,
            index: 0,
            checksum_acc: 0,
            crc_acc: 0,
            crc_expected: 0,
            params: [0u8; MAX_PARAMS],
        }
    }

    /// Reset internal state machine.
    pub fn reset(&mut self) {
        self.state = ParserState::WaitHeader1;
        self.id = 0;
        self.instruction = 0;
        self.length = 0;
        self.index = 0;
        self.checksum_acc = 0;
        self.crc_acc = 0;
        self.crc_expected = 0;
    }

    fn update_crc(&mut self, byte: u8) {
        self.crc_acc ^= (byte as u16) << 8;
        for _ in 0..8 {
            if (self.crc_acc & 0x8000) != 0 {
                self.crc_acc = (self.crc_acc << 1) ^ 0x8005;
            } else {
                self.crc_acc <<= 1;
            }
        }
    }

    /// Ingest a single byte from the serial stream. Returns `Some(packet)` when a complete,
    /// valid frame has been decoded.
    pub fn parse_byte(&mut self, byte: u8) -> Option<ServoPacket<MAX_PARAMS>> {
        match self.state {
            ParserState::WaitHeader1 => {
                if byte == 0xFF {
                    self.state = ParserState::WaitHeader2;
                }
            }
            ParserState::WaitHeader2 => {
                if byte == 0xFF {
                    self.state = ParserState::CheckVersionOrId;
                } else {
                    self.state = ParserState::WaitHeader1;
                }
            }
            ParserState::CheckVersionOrId => {
                if byte == 0xFD {
                    self.state = ParserState::V2HeaderReserved;
                } else if byte != 0xFF {
                    // Protocol V1 ID
                    self.id = byte;
                    self.checksum_acc = byte;
                    self.state = ParserState::V1Length;
                }
            }
            // --- V1 Flow ---
            ParserState::V1Length => {
                if byte < 2 {
                    self.reset();
                } else {
                    self.length = byte as usize;
                    self.checksum_acc = self.checksum_acc.wrapping_add(byte);
                    self.state = ParserState::V1Instruction;
                }
            }
            ParserState::V1Instruction => {
                self.instruction = byte;
                self.checksum_acc = self.checksum_acc.wrapping_add(byte);
                self.index = 0;
                let needed_params = self.length.saturating_sub(2);
                if needed_params == 0 {
                    self.state = ParserState::V1Checksum;
                } else if needed_params <= MAX_PARAMS {
                    self.state = ParserState::V1Params;
                } else {
                    self.reset();
                }
            }
            ParserState::V1Params => {
                self.params[self.index] = byte;
                self.index += 1;
                self.checksum_acc = self.checksum_acc.wrapping_add(byte);
                let needed_params = self.length.saturating_sub(2);
                if self.index >= needed_params {
                    self.state = ParserState::V1Checksum;
                }
            }
            ParserState::V1Checksum => {
                let expected = !self.checksum_acc;
                self.state = ParserState::WaitHeader1;
                if byte == expected {
                    return Some(ServoPacket {
                        protocol: ServoProtocolKind::V1,
                        id: self.id,
                        instruction: self.instruction,
                        params: self.params,
                        params_len: self.index,
                    });
                }
            }
            // --- V2 Flow ---
            ParserState::V2HeaderReserved => {
                if byte == 0x00 {
                    self.crc_acc = 0;
                    self.update_crc(0xFF);
                    self.update_crc(0xFF);
                    self.update_crc(0xFD);
                    self.update_crc(0x00);
                    self.state = ParserState::V2Id;
                } else {
                    self.reset();
                }
            }
            ParserState::V2Id => {
                self.id = byte;
                self.update_crc(byte);
                self.state = ParserState::V2LengthL;
            }
            ParserState::V2LengthL => {
                self.length = byte as usize;
                self.update_crc(byte);
                self.state = ParserState::V2LengthH;
            }
            ParserState::V2LengthH => {
                self.length |= (byte as usize) << 8;
                self.update_crc(byte);
                if self.length < 3 {
                    self.reset();
                } else {
                    self.state = ParserState::V2Instruction;
                }
            }
            ParserState::V2Instruction => {
                self.instruction = byte;
                self.update_crc(byte);
                self.index = 0;
                let needed_params = self.length.saturating_sub(3);
                if needed_params == 0 {
                    self.state = ParserState::V2CrcL;
                } else if needed_params <= MAX_PARAMS {
                    self.state = ParserState::V2Params;
                } else {
                    self.reset();
                }
            }
            ParserState::V2Params => {
                self.params[self.index] = byte;
                self.index += 1;
                self.update_crc(byte);
                let needed_params = self.length.saturating_sub(3);
                if self.index >= needed_params {
                    self.state = ParserState::V2CrcL;
                }
            }
            ParserState::V2CrcL => {
                self.crc_expected = byte as u16;
                self.state = ParserState::V2CrcH;
            }
            ParserState::V2CrcH => {
                self.crc_expected |= (byte as u16) << 8;
                self.state = ParserState::WaitHeader1;
                if self.crc_acc == self.crc_expected {
                    return Some(ServoPacket {
                        protocol: ServoProtocolKind::V2,
                        id: self.id,
                        instruction: self.instruction,
                        params: self.params,
                        params_len: self.index,
                    });
                }
            }
        }
        None
    }
}

/// Target parsed from a SYNC_WRITE packet.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AtomicSyncSetpoint {
    pub joint_index: usize,
    pub setpoint_rad: f32,
}

/// Decodes atomic SYNC_WRITE setpoints targeting local joints on this board.
pub fn parse_sync_write_setpoints<const N: usize, const MAX_PARAMS: usize>(
    packet: &ServoPacket<MAX_PARAMS>,
    router: &MultiIdRouter<N>,
    output_setpoints: &mut [AtomicSyncSetpoint],
) -> usize {
    if packet.instruction != INST_SYNC_WRITE {
        return 0;
    }

    let params = packet.params();
    let mut out_count = 0;

    match packet.protocol {
        ServoProtocolKind::V1 => {
            if params.len() < 2 {
                return 0;
            }
            let _start_addr = params[0];
            let data_len = params[1] as usize;
            if data_len == 0 {
                return 0;
            }

            let stride = 1 + data_len;
            let mut offset = 2;

            while offset + stride <= params.len() {
                let servo_id = params[offset];
                let payload = &params[offset + 1..offset + stride];

                if let RouteTarget::Joint(joint_idx) = router.resolve(servo_id) {
                    if let Some(target_val) = decode_setpoint(payload) {
                        if out_count < output_setpoints.len() {
                            output_setpoints[out_count] = AtomicSyncSetpoint {
                                joint_index: joint_idx,
                                setpoint_rad: target_val,
                            };
                            out_count += 1;
                        }
                    }
                }
                offset += stride;
            }
        }
        ServoProtocolKind::V2 => {
            if params.len() < 4 {
                return 0;
            }
            let _start_addr = (params[0] as u16) | ((params[1] as u16) << 8);
            let data_len = ((params[2] as u16) | ((params[3] as u16) << 8)) as usize;
            if data_len == 0 {
                return 0;
            }

            let stride = 1 + data_len;
            let mut offset = 4;

            while offset + stride <= params.len() {
                let servo_id = params[offset];
                let payload = &params[offset + 1..offset + stride];

                if let RouteTarget::Joint(joint_idx) = router.resolve(servo_id) {
                    if let Some(target_val) = decode_setpoint(payload) {
                        if out_count < output_setpoints.len() {
                            output_setpoints[out_count] = AtomicSyncSetpoint {
                                joint_index: joint_idx,
                                setpoint_rad: target_val,
                            };
                            out_count += 1;
                        }
                    }
                }
                offset += stride;
            }
        }
    }

    out_count
}

fn decode_setpoint(bytes: &[u8]) -> Option<f32> {
    if bytes.len() >= 4 {
        Some(f32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]))
    } else if bytes.len() >= 2 {
        let raw = (bytes[0] as i16) | ((bytes[1] as i16) << 8);
        Some(raw as f32)
    } else {
        None
    }
}

#[cfg(feature = "kinematics")]
pub fn apply_sync_write_to_assembly<const M: usize, const S: usize, const N: usize, const MAX_PARAMS: usize>(
    packet: &ServoPacket<MAX_PARAMS>,
    router: &MultiIdRouter<N>,
    assembly: &mut barefoc::kinematics::ActuatorAssembly<M, S>,
) -> usize {
    let mut targets = [AtomicSyncSetpoint { joint_index: 0, setpoint_rad: 0.0 }; N];
    let count = parse_sync_write_setpoints(packet, router, &mut targets);
    for setpoint in &targets[..count] {
        assembly.set_joint_target(setpoint.joint_index, setpoint.setpoint_rad);
    }
    count
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_v1_streaming_parser_and_sync_write() {
        let mut parser = StreamingServoParser::<64>::new();
        let router = MultiIdRouter::new(10, [1, 2, 3, 4]);

        // Build V1 SYNC_WRITE frame for ID 1 and ID 2
        // Param 0: Start Address 0x70
        // Param 1: Data Length 4
        // Item 1: ID 1, value 1.5708f32
        // Item 2: ID 2, value -3.1415f32
        let val1 = 1.5708f32.to_le_bytes();
        let val2 = (-3.1415f32).to_le_bytes();

        let mut packet_bytes = [
            0xFF, 0xFF,
            0xFE, // Broadcast ID
            14,   // Length: 12 params + 2 = 14
            INST_SYNC_WRITE,
            0x70, // Start addr
            0x04, // Data len per servo
            1, val1[0], val1[1], val1[2], val1[3],
            2, val2[0], val2[1], val2[2], val2[3],
            0, // checksum placeholder
        ];

        let mut sum: u8 = 0;
        for &b in &packet_bytes[2..packet_bytes.len() - 1] {
            sum = sum.wrapping_add(b);
        }
        let last_idx = packet_bytes.len() - 1;
        packet_bytes[last_idx] = !sum;

        let mut parsed_packet = None;
        for &b in &packet_bytes {
            if let Some(p) = parser.parse_byte(b) {
                parsed_packet = Some(p);
            }
        }

        let packet = parsed_packet.expect("Failed to parse V1 SYNC_WRITE frame");
        assert_eq!(packet.protocol, ServoProtocolKind::V1);
        assert_eq!(packet.instruction, INST_SYNC_WRITE);

        let mut setpoints = [AtomicSyncSetpoint { joint_index: 0, setpoint_rad: 0.0 }; 4];
        let count = parse_sync_write_setpoints(&packet, &router, &mut setpoints);
        assert_eq!(count, 2);
        assert_eq!(setpoints[0].joint_index, 0);
        assert!((setpoints[0].setpoint_rad - 1.5708).abs() < 1e-4);
        assert_eq!(setpoints[1].joint_index, 1);
        assert!((setpoints[1].setpoint_rad - (-3.1415)).abs() < 1e-4);
    }

    #[test]
    fn test_v2_streaming_parser_and_sync_write() {
        let mut parser = StreamingServoParser::<64>::new();
        let router = MultiIdRouter::new(10, [1, 2, 3, 4]);

        let val1 = 0.7854f32.to_le_bytes();
        let val2 = 2.3562f32.to_le_bytes();

        let mut raw_bytes = std::vec::Vec::new();
        raw_bytes.extend_from_slice(&[0xFF, 0xFF, 0xFD, 0x00]);
        raw_bytes.push(0xFE); // Broadcast ID
        let length: u16 = 1 + 2 + 2 + (1 + 4) * 2 + 2; // Inst(1) + Addr(2) + Len(2) + 2*(1+4) + CRC(2) = 17
        raw_bytes.extend_from_slice(&length.to_le_bytes());
        raw_bytes.push(INST_SYNC_WRITE);
        raw_bytes.extend_from_slice(&0x0070u16.to_le_bytes()); // Start addr
        raw_bytes.extend_from_slice(&0x0004u16.to_le_bytes()); // Data len
        raw_bytes.push(1);
        raw_bytes.extend_from_slice(&val1);
        raw_bytes.push(2);
        raw_bytes.extend_from_slice(&val2);

        let crc = calc_dxl2_crc(&raw_bytes);
        raw_bytes.extend_from_slice(&crc.to_le_bytes());

        let mut parsed_packet = None;
        for &b in &raw_bytes {
            if let Some(p) = parser.parse_byte(b) {
                parsed_packet = Some(p);
            }
        }

        let packet = parsed_packet.expect("Failed to parse V2 SYNC_WRITE frame");
        assert_eq!(packet.protocol, ServoProtocolKind::V2);
        assert_eq!(packet.instruction, INST_SYNC_WRITE);

        let mut setpoints = [AtomicSyncSetpoint { joint_index: 0, setpoint_rad: 0.0 }; 4];
        let count = parse_sync_write_setpoints(&packet, &router, &mut setpoints);
        assert_eq!(count, 2);
        assert_eq!(setpoints[0].joint_index, 0);
        assert!((setpoints[0].setpoint_rad - 0.7854).abs() < 1e-4);
        assert_eq!(setpoints[1].joint_index, 1);
        assert!((setpoints[1].setpoint_rad - 2.3562).abs() < 1e-4);
    }

    #[cfg(feature = "kinematics")]
    #[test]
    fn test_apply_sync_write_to_assembly() {
        let router = MultiIdRouter::new(10, [1, 2, 3, 4]);
        let mut assembly = barefoc::kinematics::ActuatorAssembly::<4, 4>::new();

        let val1 = 1.234f32.to_le_bytes();
        let val2 = (-2.345f32).to_le_bytes();

        let mut raw_bytes = std::vec::Vec::new();
        raw_bytes.extend_from_slice(&[0xFF, 0xFF, 0xFD, 0x00]);
        raw_bytes.push(0xFE); // Broadcast ID
        let length: u16 = 1 + 2 + 2 + (1 + 4) * 2 + 2;
        raw_bytes.extend_from_slice(&length.to_le_bytes());
        raw_bytes.push(INST_SYNC_WRITE);
        raw_bytes.extend_from_slice(&0x0070u16.to_le_bytes()); // Start addr
        raw_bytes.extend_from_slice(&0x0004u16.to_le_bytes()); // Data len
        raw_bytes.push(1);
        raw_bytes.extend_from_slice(&val1);
        raw_bytes.push(2);
        raw_bytes.extend_from_slice(&val2);

        let crc = calc_dxl2_crc(&raw_bytes);
        raw_bytes.extend_from_slice(&crc.to_le_bytes());

        let mut parser = StreamingServoParser::<64>::new();
        let mut parsed_packet = None;
        for &b in &raw_bytes {
            if let Some(p) = parser.parse_byte(b) {
                parsed_packet = Some(p);
            }
        }

        let packet = parsed_packet.expect("Failed to parse packet");
        let count = apply_sync_write_to_assembly(&packet, &router, &mut assembly);
        assert_eq!(count, 2);
        assert!((assembly.joints[0].target_pos - 1.234).abs() < 1e-4);
        assert!((assembly.joints[1].target_pos - (-2.345)).abs() < 1e-4);
    }
}
