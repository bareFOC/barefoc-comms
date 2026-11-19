//! ODrive GUI and odrivetool compatibility schema and endpoint mappings.
//!
//! Provides preconfigured schemas and endpoint definitions mapping
//! `barefoc::motor::MotorConfig` to the canonical ODrive interface:
//! - `vbus_voltage`: DC bus voltage
//! - `axis0.target_velocity`: target velocity setpoint
//! - `axis0.current_q`: measured quadrature current
//! - `axis0.shaft_angle`: current mechanical shaft angle
//! - `axis0.shaft_velocity`: current shaft velocity
//!
//! References upstream:
//! - `ref/fibre-cpp/legacy_protocol.hpp`
//! - `ref/fibre-cpp/legacy_object_client.cpp`

use crate::schema::Endpoint0Handler;
use crate::server::Endpoint;

#[cfg(feature = "barefoc")]
use barefoc::common::time::Clock;
#[cfg(feature = "barefoc")]
use barefoc::motor::base::MotorConfig;

/// Canonical minified JSON interface manifest for ODrive GUI / odrivetool.
pub const BAREFOC_ODRIVE_JSON: &str = r#"[{"name":"vbus_voltage","id":1,"type":"float","access":"r"},{"name":"axis0","type":"object","members":[{"name":"target_velocity","id":2,"type":"float","access":"rw"},{"name":"current_q","id":3,"type":"float","access":"r"},{"name":"shaft_angle","id":4,"type":"float","access":"r"},{"name":"shaft_velocity","id":5,"type":"float","access":"r"}]}]"#;

/// Returns an `Endpoint0Handler` initialized with `BAREFOC_ODRIVE_JSON`.
pub fn create_odrive_schema() -> Endpoint0Handler {
    Endpoint0Handler::new(BAREFOC_ODRIVE_JSON)
}

/// Canonical minified JSON interface manifest for 4-axis kinematics ODrive GUI / odrivetool.
///
/// Exposes:
/// - `vbus_voltage`, `topology_mode`
/// - `axis0` through `axis3` (`target`, `current`, `shaft_angle`, `shaft_velocity`, `sensor_channel`)
/// - `joint0` through `joint3` (`target_pos`, `present_pos`, `present_vel`, `gear_ratio`, `backlash`, `pid_p`, `pid_i`, `pid_d`)
/// - `calib` (`command`, `state`, `progress`, `flash_status`)
pub const BAREFOC_4AXIS_KINEMATICS_JSON: &str = r#"[{"name":"vbus_voltage","id":1,"type":"float","access":"r"},{"name":"topology_mode","id":2,"type":"uint32","access":"rw"},{"name":"axis0","type":"object","members":[{"name":"target","id":3,"type":"float","access":"rw"},{"name":"current","id":4,"type":"float","access":"r"},{"name":"shaft_angle","id":5,"type":"float","access":"r"},{"name":"shaft_velocity","id":6,"type":"float","access":"r"},{"name":"sensor_channel","id":7,"type":"uint32","access":"rw"}]},{"name":"axis1","type":"object","members":[{"name":"target","id":8,"type":"float","access":"rw"},{"name":"current","id":9,"type":"float","access":"r"},{"name":"shaft_angle","id":10,"type":"float","access":"r"},{"name":"shaft_velocity","id":11,"type":"float","access":"r"},{"name":"sensor_channel","id":12,"type":"uint32","access":"rw"}]},{"name":"axis2","type":"object","members":[{"name":"target","id":13,"type":"float","access":"rw"},{"name":"current","id":14,"type":"float","access":"r"},{"name":"shaft_angle","id":15,"type":"float","access":"r"},{"name":"shaft_velocity","id":16,"type":"float","access":"r"},{"name":"sensor_channel","id":17,"type":"uint32","access":"rw"}]},{"name":"axis3","type":"object","members":[{"name":"target","id":18,"type":"float","access":"rw"},{"name":"current","id":19,"type":"float","access":"r"},{"name":"shaft_angle","id":20,"type":"float","access":"r"},{"name":"shaft_velocity","id":21,"type":"float","access":"r"},{"name":"sensor_channel","id":22,"type":"uint32","access":"rw"}]},{"name":"joint0","type":"object","members":[{"name":"target_pos","id":23,"type":"float","access":"rw"},{"name":"present_pos","id":24,"type":"float","access":"r"},{"name":"present_vel","id":25,"type":"float","access":"r"},{"name":"gear_ratio","id":26,"type":"float","access":"rw"},{"name":"backlash","id":27,"type":"float","access":"rw"},{"name":"pid_p","id":28,"type":"float","access":"rw"},{"name":"pid_i","id":29,"type":"float","access":"rw"},{"name":"pid_d","id":30,"type":"float","access":"rw"}]},{"name":"joint1","type":"object","members":[{"name":"target_pos","id":31,"type":"float","access":"rw"},{"name":"present_pos","id":32,"type":"float","access":"r"},{"name":"present_vel","id":33,"type":"float","access":"r"},{"name":"gear_ratio","id":34,"type":"float","access":"rw"},{"name":"backlash","id":35,"type":"float","access":"rw"},{"name":"pid_p","id":36,"type":"float","access":"rw"},{"name":"pid_i","id":37,"type":"float","access":"rw"},{"name":"pid_d","id":38,"type":"float","access":"rw"}]},{"name":"joint2","type":"object","members":[{"name":"target_pos","id":39,"type":"float","access":"rw"},{"name":"present_pos","id":40,"type":"float","access":"r"},{"name":"present_vel","id":41,"type":"float","access":"r"},{"name":"gear_ratio","id":42,"type":"float","access":"rw"},{"name":"backlash","id":43,"type":"float","access":"rw"},{"name":"pid_p","id":44,"type":"float","access":"rw"},{"name":"pid_i","id":45,"type":"float","access":"rw"},{"name":"pid_d","id":46,"type":"float","access":"rw"}]},{"name":"joint3","type":"object","members":[{"name":"target_pos","id":47,"type":"float","access":"rw"},{"name":"present_pos","id":48,"type":"float","access":"r"},{"name":"present_vel","id":49,"type":"float","access":"r"},{"name":"gear_ratio","id":50,"type":"float","access":"rw"},{"name":"backlash","id":51,"type":"float","access":"rw"},{"name":"pid_p","id":52,"type":"float","access":"rw"},{"name":"pid_i","id":53,"type":"float","access":"rw"},{"name":"pid_d","id":54,"type":"float","access":"rw"}]},{"name":"calib","type":"object","members":[{"name":"command","id":55,"type":"uint32","access":"rw"},{"name":"state","id":56,"type":"uint32","access":"r"},{"name":"progress","id":57,"type":"float","access":"r"},{"name":"flash_status","id":58,"type":"uint32","access":"r"}]}]"#;

/// Returns an `Endpoint0Handler` initialized with `BAREFOC_4AXIS_KINEMATICS_JSON`.
pub fn create_4axis_kinematics_schema() -> Endpoint0Handler {
    Endpoint0Handler::new(BAREFOC_4AXIS_KINEMATICS_JSON)
}

/// State of a single motor axis exposed via Fibre RPC.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AxisState {
    /// Target setpoint (velocity, angle, or torque).
    pub target: f32,
    /// Measured current in Amperes (e.g. Iq or DC current).
    pub current: f32,
    /// Measured mechanical shaft angle in radians.
    pub shaft_angle: f32,
    /// Measured shaft velocity in radians per second.
    pub shaft_velocity: f32,
    /// Multiplexed sensor hardware channel index (0..3).
    pub sensor_channel: u32,
}

impl Default for AxisState {
    fn default() -> Self {
        Self {
            target: 0.0,
            current: 0.0,
            shaft_angle: 0.0,
            shaft_velocity: 0.0,
            sensor_channel: 0,
        }
    }
}

/// State of a single kinematic joint exposed via Fibre RPC.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct JointState {
    /// Outer joint target position in radians or millimeters.
    pub target_pos: f32,
    /// Measured joint present position.
    pub present_pos: f32,
    /// Measured joint present velocity.
    pub present_vel: f32,
    /// Transmission gear ratio.
    pub gear_ratio: f32,
    /// Backlash compensation value.
    pub backlash: f32,
    /// Outer position controller proportional gain Kp.
    pub pid_p: f32,
    /// Outer position controller integral gain Ki.
    pub pid_i: f32,
    /// Outer position controller derivative gain Kd.
    pub pid_d: f32,
}

impl Default for JointState {
    fn default() -> Self {
        Self {
            target_pos: 0.0,
            present_pos: 0.0,
            present_vel: 0.0,
            gear_ratio: 1.0,
            backlash: 0.0,
            pid_p: 1.0,
            pid_i: 0.0,
            pid_d: 0.0,
        }
    }
}

/// Auto-calibration state and status exposed via Fibre RPC.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct CalibState {
    /// Active calibration command or trigger code.
    pub command: u32,
    /// Calibration state machine phase / state ID.
    pub state: u32,
    /// Progress fraction [0.0..1.0].
    pub progress: f32,
    /// Flash storage persistence status code.
    pub flash_status: u32,
}

/// Multi-axis actuator assembly state for Fibre RPC server.
///
/// Encapsulates DC bus voltage, mechanism topology mode, 4 motor axes,
/// 4 kinematic joints, and on-device auto-calibration telemetry.
#[derive(Debug, Clone, PartialEq)]
pub struct ActuatorAssembly {
    /// Measured DC bus voltage in Volts.
    pub vbus_voltage: f32,
    /// Active mechanism topology preset ID (e.g. 0=Four1Dof, 1=Two2Dof, etc.).
    pub topology_mode: u32,
    /// 4 motor axis states.
    pub axes: [AxisState; 4],
    /// 4 kinematic joint states.
    pub joints: [JointState; 4],
    /// Calibration supervisor state and status.
    pub calib: CalibState,
}

impl Default for ActuatorAssembly {
    fn default() -> Self {
        Self {
            vbus_voltage: 0.0,
            topology_mode: 0,
            axes: [
                AxisState { sensor_channel: 0, ..AxisState::default() },
                AxisState { sensor_channel: 1, ..AxisState::default() },
                AxisState { sensor_channel: 2, ..AxisState::default() },
                AxisState { sensor_channel: 3, ..AxisState::default() },
            ],
            joints: [
                JointState::default(),
                JointState::default(),
                JointState::default(),
                JointState::default(),
            ],
            calib: CalibState::default(),
        }
    }
}

impl ActuatorAssembly {
    /// Creates a new `ActuatorAssembly` with default parameters.
    pub fn new() -> Self {
        Self::default()
    }
}

/// Creates an endpoint registry mapping `BAREFOC_4AXIS_KINEMATICS_JSON` to `ActuatorAssembly`.
pub fn make_4axis_assembly_endpoints() -> [Endpoint<ActuatorAssembly>; 58] {
    [
        // 1..2: Global system
        Endpoint::readonly_f32(1, |a| a.vbus_voltage),
        Endpoint::readwrite_u32(2, |a| a.topology_mode, |a, v| a.topology_mode = v),

        // 3..7: Axis 0
        Endpoint::readwrite_f32(3, |a| a.axes[0].target, |a, v| a.axes[0].target = v),
        Endpoint::readonly_f32(4, |a| a.axes[0].current),
        Endpoint::readonly_f32(5, |a| a.axes[0].shaft_angle),
        Endpoint::readonly_f32(6, |a| a.axes[0].shaft_velocity),
        Endpoint::readwrite_u32(7, |a| a.axes[0].sensor_channel, |a, v| a.axes[0].sensor_channel = v),

        // 8..12: Axis 1
        Endpoint::readwrite_f32(8, |a| a.axes[1].target, |a, v| a.axes[1].target = v),
        Endpoint::readonly_f32(9, |a| a.axes[1].current),
        Endpoint::readonly_f32(10, |a| a.axes[1].shaft_angle),
        Endpoint::readonly_f32(11, |a| a.axes[1].shaft_velocity),
        Endpoint::readwrite_u32(12, |a| a.axes[1].sensor_channel, |a, v| a.axes[1].sensor_channel = v),

        // 13..17: Axis 2
        Endpoint::readwrite_f32(13, |a| a.axes[2].target, |a, v| a.axes[2].target = v),
        Endpoint::readonly_f32(14, |a| a.axes[2].current),
        Endpoint::readonly_f32(15, |a| a.axes[2].shaft_angle),
        Endpoint::readonly_f32(16, |a| a.axes[2].shaft_velocity),
        Endpoint::readwrite_u32(17, |a| a.axes[2].sensor_channel, |a, v| a.axes[2].sensor_channel = v),

        // 18..22: Axis 3
        Endpoint::readwrite_f32(18, |a| a.axes[3].target, |a, v| a.axes[3].target = v),
        Endpoint::readonly_f32(19, |a| a.axes[3].current),
        Endpoint::readonly_f32(20, |a| a.axes[3].shaft_angle),
        Endpoint::readonly_f32(21, |a| a.axes[3].shaft_velocity),
        Endpoint::readwrite_u32(22, |a| a.axes[3].sensor_channel, |a, v| a.axes[3].sensor_channel = v),

        // 23..30: Joint 0
        Endpoint::readwrite_f32(23, |a| a.joints[0].target_pos, |a, v| a.joints[0].target_pos = v),
        Endpoint::readonly_f32(24, |a| a.joints[0].present_pos),
        Endpoint::readonly_f32(25, |a| a.joints[0].present_vel),
        Endpoint::readwrite_f32(26, |a| a.joints[0].gear_ratio, |a, v| a.joints[0].gear_ratio = v),
        Endpoint::readwrite_f32(27, |a| a.joints[0].backlash, |a, v| a.joints[0].backlash = v),
        Endpoint::readwrite_f32(28, |a| a.joints[0].pid_p, |a, v| a.joints[0].pid_p = v),
        Endpoint::readwrite_f32(29, |a| a.joints[0].pid_i, |a, v| a.joints[0].pid_i = v),
        Endpoint::readwrite_f32(30, |a| a.joints[0].pid_d, |a, v| a.joints[0].pid_d = v),

        // 31..38: Joint 1
        Endpoint::readwrite_f32(31, |a| a.joints[1].target_pos, |a, v| a.joints[1].target_pos = v),
        Endpoint::readonly_f32(32, |a| a.joints[1].present_pos),
        Endpoint::readonly_f32(33, |a| a.joints[1].present_vel),
        Endpoint::readwrite_f32(34, |a| a.joints[1].gear_ratio, |a, v| a.joints[1].gear_ratio = v),
        Endpoint::readwrite_f32(35, |a| a.joints[1].backlash, |a, v| a.joints[1].backlash = v),
        Endpoint::readwrite_f32(36, |a| a.joints[1].pid_p, |a, v| a.joints[1].pid_p = v),
        Endpoint::readwrite_f32(37, |a| a.joints[1].pid_i, |a, v| a.joints[1].pid_i = v),
        Endpoint::readwrite_f32(38, |a| a.joints[1].pid_d, |a, v| a.joints[1].pid_d = v),

        // 39..46: Joint 2
        Endpoint::readwrite_f32(39, |a| a.joints[2].target_pos, |a, v| a.joints[2].target_pos = v),
        Endpoint::readonly_f32(40, |a| a.joints[2].present_pos),
        Endpoint::readonly_f32(41, |a| a.joints[2].present_vel),
        Endpoint::readwrite_f32(42, |a| a.joints[2].gear_ratio, |a, v| a.joints[2].gear_ratio = v),
        Endpoint::readwrite_f32(43, |a| a.joints[2].backlash, |a, v| a.joints[2].backlash = v),
        Endpoint::readwrite_f32(44, |a| a.joints[2].pid_p, |a, v| a.joints[2].pid_p = v),
        Endpoint::readwrite_f32(45, |a| a.joints[2].pid_i, |a, v| a.joints[2].pid_i = v),
        Endpoint::readwrite_f32(46, |a| a.joints[2].pid_d, |a, v| a.joints[2].pid_d = v),

        // 47..54: Joint 3
        Endpoint::readwrite_f32(47, |a| a.joints[3].target_pos, |a, v| a.joints[3].target_pos = v),
        Endpoint::readonly_f32(48, |a| a.joints[3].present_pos),
        Endpoint::readonly_f32(49, |a| a.joints[3].present_vel),
        Endpoint::readwrite_f32(50, |a| a.joints[3].gear_ratio, |a, v| a.joints[3].gear_ratio = v),
        Endpoint::readwrite_f32(51, |a| a.joints[3].backlash, |a, v| a.joints[3].backlash = v),
        Endpoint::readwrite_f32(52, |a| a.joints[3].pid_p, |a, v| a.joints[3].pid_p = v),
        Endpoint::readwrite_f32(53, |a| a.joints[3].pid_i, |a, v| a.joints[3].pid_i = v),
        Endpoint::readwrite_f32(54, |a| a.joints[3].pid_d, |a, v| a.joints[3].pid_d = v),

        // 55..58: Calib
        Endpoint::readwrite_u32(55, |a| a.calib.command, |a, v| a.calib.command = v),
        Endpoint::readonly_u32(56, |a| a.calib.state),
        Endpoint::readonly_f32(57, |a| a.calib.progress),
        Endpoint::readonly_u32(58, |a| a.calib.flash_status),
    ]
}

#[cfg(feature = "barefoc")]
/// Combined context holding motor configuration and measured bus voltage.
pub struct BareFocAxisContext<'a, C: Clock> {
    /// Measured DC bus voltage.
    pub vbus_voltage: f32,
    /// Reference to the BareFOC motor configuration state.
    pub motor: &'a mut MotorConfig<C>,
}

#[cfg(feature = "barefoc")]
/// Creates an endpoint registry mapping `BAREFOC_ODRIVE_JSON` to `MotorConfig<C>`.
///
/// In this mapping, `vbus_voltage` is mapped to `motor.voltage_limit`.
pub fn make_motor_config_endpoints<C: Clock>() -> [Endpoint<MotorConfig<C>>; 5] {
    [
        Endpoint::readonly_f32(1, |m| m.voltage_limit),
        Endpoint::readwrite_f32(2, |m| m.target, |m, v| m.target = v),
        Endpoint::readonly_f32(3, |m| m.current.q),
        Endpoint::readonly_f32(4, |m| m.shaft_angle),
        Endpoint::readonly_f32(5, |m| m.shaft_velocity),
    ]
}

#[cfg(feature = "barefoc")]
/// Creates an endpoint registry mapping `BAREFOC_ODRIVE_JSON` to `BareFocAxisContext`.
pub fn make_axis_context_endpoints<'a, C: Clock>() -> [Endpoint<BareFocAxisContext<'a, C>>; 5] {
    [
        Endpoint::readonly_f32(1, |ctx| ctx.vbus_voltage),
        Endpoint::readwrite_f32(2, |ctx| ctx.motor.target, |ctx, v| ctx.motor.target = v),
        Endpoint::readonly_f32(3, |ctx| ctx.motor.current.q),
        Endpoint::readonly_f32(4, |ctx| ctx.motor.shaft_angle),
        Endpoint::readonly_f32(5, |ctx| ctx.motor.shaft_velocity),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::server::FibreServer;
    use barefoc::motor::base::{MotorConfig, NullClock};

    #[test]
    fn test_odrive_schema_initialization() {
        let schema = create_odrive_schema();
        assert_ne!(schema.json_crc(), 0);
        assert_eq!(schema.json_str(), BAREFOC_ODRIVE_JSON);
    }

    #[test]
    fn test_motor_config_endpoints_integration() {
        let schema = create_odrive_schema();
        let endpoints = make_motor_config_endpoints::<NullClock>();
        let server = FibreServer::new(schema, &endpoints);

        let clock = NullClock;
        let mut config = MotorConfig::new(&clock);
        config.voltage_limit = 24.0;
        config.target = 10.0;
        config.current.q = 1.25;
        config.shaft_angle = core::f32::consts::PI;
        config.shaft_velocity = 50.0;

        // Read vbus_voltage (ep 1)
        let mut req = [0u8; 8];
        req[0..2].copy_from_slice(&1u16.to_le_bytes());
        req[2..4].copy_from_slice(&(1u16 | 0x8000).to_le_bytes());
        req[4..6].copy_from_slice(&4u16.to_le_bytes());
        req[6..8].copy_from_slice(&schema.json_crc().to_le_bytes());

        let mut resp = [0u8; 32];
        let len = server.process_packet(&req, &mut resp, &mut config).unwrap().unwrap();
        assert_eq!(len, 6);
        let vbus = f32::from_le_bytes([resp[2], resp[3], resp[4], resp[5]]);
        assert_eq!(vbus, 24.0);

        // Write target_velocity (ep 2)
        let mut write_req = [0u8; 12];
        write_req[0..2].copy_from_slice(&2u16.to_le_bytes());
        write_req[2..4].copy_from_slice(&(2u16 | 0x8000).to_le_bytes());
        write_req[4..6].copy_from_slice(&4u16.to_le_bytes());
        write_req[6..10].copy_from_slice(&75.5f32.to_le_bytes());
        write_req[10..12].copy_from_slice(&schema.json_crc().to_le_bytes());

        let write_len = server
            .process_packet(&write_req, &mut resp, &mut config)
            .unwrap()
            .unwrap();
        assert_eq!(write_len, 6);
        assert_eq!(config.target, 75.5);
    }

    #[test]
    fn test_4axis_schema_initialization() {
        let schema = create_4axis_kinematics_schema();
        assert_ne!(schema.json_crc(), 0);
        assert_eq!(schema.json_str(), BAREFOC_4AXIS_KINEMATICS_JSON);

        // Test Endpoint 0 chunked schema reads
        let endpoints = make_4axis_assembly_endpoints();
        let server = FibreServer::new(schema, &endpoints);
        let mut assembly = ActuatorAssembly::new();

        // Read first 64 bytes of schema from Endpoint 0
        let mut req = [0u8; 12];
        req[0..2].copy_from_slice(&1u16.to_le_bytes()); // seq_no
        req[2..4].copy_from_slice(&0x8000u16.to_le_bytes()); // ep 0 with expect_response
        req[4..6].copy_from_slice(&64u16.to_le_bytes()); // expected rx length
        req[6..10].copy_from_slice(&0u32.to_le_bytes()); // offset 0
        req[10..12].copy_from_slice(&crate::crc::PROTOCOL_VERSION.to_le_bytes()); // trailer

        let mut resp = [0u8; 128];
        let resp_len = server
            .process_packet(&req, &mut resp, &mut assembly)
            .unwrap()
            .unwrap();

        assert_eq!(resp_len, 2 + 64);
        assert_eq!(&resp[2..2 + 64], &BAREFOC_4AXIS_KINEMATICS_JSON.as_bytes()[..64]);
    }

    #[test]
    fn test_4axis_assembly_endpoints_integration() {
        let schema = create_4axis_kinematics_schema();
        let endpoints = make_4axis_assembly_endpoints();
        assert_eq!(endpoints.len(), 58);
        let server = FibreServer::new(schema, &endpoints);

        let mut assembly = ActuatorAssembly::new();
        assembly.vbus_voltage = 28.5;
        assembly.topology_mode = 2;
        assembly.axes[0].target = 10.0;
        assembly.axes[0].current = 1.5;
        assembly.axes[0].shaft_angle = 3.14159;
        assembly.axes[0].shaft_velocity = 25.0;
        assembly.axes[0].sensor_channel = 1;

        assembly.joints[0].target_pos = 1.0;
        assembly.joints[0].present_pos = 0.785;
        assembly.joints[0].present_vel = 0.1;
        assembly.joints[0].gear_ratio = 10.0;
        assembly.joints[0].backlash = 0.02;
        assembly.joints[0].pid_p = 5.0;
        assembly.joints[0].pid_i = 0.1;
        assembly.joints[0].pid_d = 0.05;

        assembly.joints[3].present_pos = -0.5;

        assembly.calib.command = 1;
        assembly.calib.state = 2;
        assembly.calib.progress = 0.5;
        assembly.calib.flash_status = 0;

        let crc = schema.json_crc();
        let mut resp = [0u8; 64];

        // 1. Read vbus_voltage (ep 1)
        let mut req = [0u8; 8];
        req[0..2].copy_from_slice(&10u16.to_le_bytes());
        req[2..4].copy_from_slice(&(1u16 | 0x8000).to_le_bytes());
        req[4..6].copy_from_slice(&4u16.to_le_bytes());
        req[6..8].copy_from_slice(&crc.to_le_bytes());

        let len = server.process_packet(&req, &mut resp, &mut assembly).unwrap().unwrap();
        assert_eq!(len, 6);
        let vbus = f32::from_le_bytes([resp[2], resp[3], resp[4], resp[5]]);
        assert!((vbus - 28.5).abs() < 1e-5);

        // 2. Write topology_mode (ep 2) to 3
        let mut req_mode = [0u8; 12];
        req_mode[0..2].copy_from_slice(&11u16.to_le_bytes());
        req_mode[2..4].copy_from_slice(&(2u16 | 0x8000).to_le_bytes());
        req_mode[4..6].copy_from_slice(&4u16.to_le_bytes());
        req_mode[6..10].copy_from_slice(&3u32.to_le_bytes());
        req_mode[10..12].copy_from_slice(&crc.to_le_bytes());

        let len = server.process_packet(&req_mode, &mut resp, &mut assembly).unwrap().unwrap();
        assert_eq!(len, 6);
        assert_eq!(assembly.topology_mode, 3);
        assert_eq!(u32::from_le_bytes([resp[2], resp[3], resp[4], resp[5]]), 3);

        // 3. Write axis0.target (ep 3)
        let mut req_target = [0u8; 12];
        req_target[0..2].copy_from_slice(&12u16.to_le_bytes());
        req_target[2..4].copy_from_slice(&(3u16 | 0x8000).to_le_bytes());
        req_target[4..6].copy_from_slice(&4u16.to_le_bytes());
        req_target[6..10].copy_from_slice(&50.0f32.to_le_bytes());
        req_target[10..12].copy_from_slice(&crc.to_le_bytes());

        server.process_packet(&req_target, &mut resp, &mut assembly).unwrap().unwrap();
        assert!((assembly.axes[0].target - 50.0).abs() < 1e-5);

        // 4. Read axis0.shaft_angle (ep 5)
        let mut req_angle = [0u8; 8];
        req_angle[0..2].copy_from_slice(&13u16.to_le_bytes());
        req_angle[2..4].copy_from_slice(&(5u16 | 0x8000).to_le_bytes());
        req_angle[4..6].copy_from_slice(&4u16.to_le_bytes());
        req_angle[6..8].copy_from_slice(&crc.to_le_bytes());

        let len = server.process_packet(&req_angle, &mut resp, &mut assembly).unwrap().unwrap();
        assert_eq!(len, 6);
        let angle = f32::from_le_bytes([resp[2], resp[3], resp[4], resp[5]]);
        assert!((angle - 3.14159).abs() < 1e-5);

        // 5. Read joint0.present_pos (ep 24)
        let mut req_pos = [0u8; 8];
        req_pos[0..2].copy_from_slice(&14u16.to_le_bytes());
        req_pos[2..4].copy_from_slice(&(24u16 | 0x8000).to_le_bytes());
        req_pos[4..6].copy_from_slice(&4u16.to_le_bytes());
        req_pos[6..8].copy_from_slice(&crc.to_le_bytes());

        let len = server.process_packet(&req_pos, &mut resp, &mut assembly).unwrap().unwrap();
        assert_eq!(len, 6);
        let pos = f32::from_le_bytes([resp[2], resp[3], resp[4], resp[5]]);
        assert!((pos - 0.785).abs() < 1e-5);

        // 6. Write joint0.pid_p (ep 28)
        let mut req_pid = [0u8; 12];
        req_pid[0..2].copy_from_slice(&15u16.to_le_bytes());
        req_pid[2..4].copy_from_slice(&(28u16 | 0x8000).to_le_bytes());
        req_pid[4..6].copy_from_slice(&4u16.to_le_bytes());
        req_pid[6..10].copy_from_slice(&12.0f32.to_le_bytes());
        req_pid[10..12].copy_from_slice(&crc.to_le_bytes());

        server.process_packet(&req_pid, &mut resp, &mut assembly).unwrap().unwrap();
        assert!((assembly.joints[0].pid_p - 12.0).abs() < 1e-5);

        // 7. Read calib.progress (ep 57)
        let mut req_prog = [0u8; 8];
        req_prog[0..2].copy_from_slice(&16u16.to_le_bytes());
        req_prog[2..4].copy_from_slice(&(57u16 | 0x8000).to_le_bytes());
        req_prog[4..6].copy_from_slice(&4u16.to_le_bytes());
        req_prog[6..8].copy_from_slice(&crc.to_le_bytes());

        let len = server.process_packet(&req_prog, &mut resp, &mut assembly).unwrap().unwrap();
        assert_eq!(len, 6);
        let prog = f32::from_le_bytes([resp[2], resp[3], resp[4], resp[5]]);
        assert!((prog - 0.5).abs() < 1e-5);

        // 8. Write to read-only endpoint (ep 24) should fail
        let mut req_ro = [0u8; 12];
        req_ro[0..2].copy_from_slice(&17u16.to_le_bytes());
        req_ro[2..4].copy_from_slice(&(24u16 | 0x8000).to_le_bytes());
        req_ro[4..6].copy_from_slice(&4u16.to_le_bytes());
        req_ro[6..10].copy_from_slice(&1.234f32.to_le_bytes());
        req_ro[10..12].copy_from_slice(&crc.to_le_bytes());

        let res = server.process_packet(&req_ro, &mut resp, &mut assembly);
        assert!(matches!(res, Err(crate::server::ServerError::ReadOnlyEndpoint(24))));
    }

    #[test]
    fn test_4axis_assembly_process_framed_roundtrip() {
        let schema = create_4axis_kinematics_schema();
        let endpoints = make_4axis_assembly_endpoints();
        let server = FibreServer::new(schema, &endpoints);
        let mut assembly = ActuatorAssembly::new();

        // Write joint1.target_pos (ep 31) via framed packet
        let mut payload = [0u8; 12];
        payload[0..2].copy_from_slice(&20u16.to_le_bytes());
        payload[2..4].copy_from_slice(&(31u16 | 0x8000).to_le_bytes());
        payload[4..6].copy_from_slice(&4u16.to_le_bytes());
        payload[6..10].copy_from_slice(&1.5708f32.to_le_bytes());
        payload[10..12].copy_from_slice(&schema.json_crc().to_le_bytes());

        let mut framed_req = [0u8; 64];
        let framed_req_len = crate::framing::PacketWrapper::wrap(&payload, &mut framed_req).unwrap();

        let mut framed_resp = [0u8; 64];
        let resp_len = server
            .process_framed_packet(&framed_req[..framed_req_len], &mut framed_resp, &mut assembly)
            .unwrap()
            .unwrap();

        assert!((assembly.joints[1].target_pos - 1.5708).abs() < 1e-4);

        let mut resp_payload = [0u8; 32];
        let resp_payload_len =
            crate::framing::PacketUnwrapper::unwrap(&framed_resp[..resp_len], &mut resp_payload).unwrap();
        assert_eq!(resp_payload_len, 6);
        let returned_val = f32::from_le_bytes([
            resp_payload[2],
            resp_payload[3],
            resp_payload[4],
            resp_payload[5],
        ]);
        assert!((returned_val - 1.5708).abs() < 1e-4);
    }
}
