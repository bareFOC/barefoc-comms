//! # Kinematics command parser and 4-axis assembly commander
//!
//! Provides ASCII console tuning, telemetry streaming, and configuration for multi-axis
//! robotic kinematics, non-linear transmissions, and topology routing.
//!
//! ## Command Hierarchy (`'K'`)
//!
//! - `KM` : Topology mode (`KM 0`..`KM 3`, `KM` get)
//! - `KS` : Sensor-to-motor channel mapping (`KS <mot>:<ch>`, `KS <mot>`, `KS`)
//! - `KX` : 2-DOF coupling bitmask (`KX <mask>`, `KX`)
//! - `KT` : Joint target position (`KT <j> <rad>`, `KT <j>`, `KT`)
//! - `KA` : Joint feedback angle query (`KA <j>`, `KA`)
//! - `KV` : Joint feedback velocity query (`KV <j>`, `KV`)
//! - `KG` : Transmission gear ratio (`KG <j> <val>`, `KG <j>`, `KG`)
//! - `KB` : Transmission backlash amplitude (`KB <j> <val>`, `KB <j>`, `KB`)
//! - `KW` : Transmission velocity threshold (`KW <j> <val>`, `KW <j>`, `KW`)
//! - `KK` : Transmission torsional stiffness (`KK <j> <val>`, `KK <j>`, `KK`)
//! - `KZ` : Transmission zero offset (`KZ <j> <val>`, `KZ <j>`, `KZ`)
//! - `KP` : Outer joint position PID P-gain (`KP <j> <val>`, `KP <j>`, `KP`)
//! - `KI` : Outer joint position PID I-gain (`KI <j> <val>`, `KI <j>`, `KI`)
//! - `KD` : Outer joint position PID D-gain (`KD <j> <val>`, `KD <j>`, `KD`)
//! - `KL` : Outer joint position PID output limit (`KL <j> <val>`, `KL <j>`, `KL`)
//! - `KR` : Outer joint position PID output ramp (`KR <j> <val>`, `KR <j>`, `KR`)
//! - `KN` : Outer joint position loop downsample / decimation (`KN <j> <int>`, `KN <j>`, `KN`)
//! - `KC` : Calibration command and status (`KC 1`=Stall Sweep, `KC 3`=Save Flash, `KC 4`=Load Flash, `KC`)
//! - `KF` : Flash configuration integrity status (`KF <int>`, `KF`)

#![allow(non_snake_case)]

#[cfg(feature = "embedded-io")]
use crate::commander::VerboseMode;

/// Dedicated single-character command ID for robotics kinematics.
pub const CMD_KINEMATICS: char = 'K';

// Subcommands under 'K'
pub const SCMD_KIN_TOPOLOGY: char = 'M';
pub const SCMD_KIN_SENSOR_MAP: char = 'S';
pub const SCMD_KIN_COUPLING_MASK: char = 'X';
pub const SCMD_KIN_JOINT_TARGET: char = 'T';
pub const SCMD_KIN_JOINT_ANGLE: char = 'A';
pub const SCMD_KIN_JOINT_VELOCITY: char = 'V';

pub const SCMD_KIN_GEAR_RATIO: char = 'G';
pub const SCMD_KIN_BACKLASH: char = 'B';
pub const SCMD_KIN_OMEGA_THRESH: char = 'W';
pub const SCMD_KIN_STIFFNESS: char = 'K';
pub const SCMD_KIN_ZERO_OFFSET: char = 'Z';

pub const SCMD_KIN_PID_P: char = 'P';
pub const SCMD_KIN_PID_I: char = 'I';
pub const SCMD_KIN_PID_D: char = 'D';
pub const SCMD_KIN_PID_LIM: char = 'L';
pub const SCMD_KIN_PID_RAMP: char = 'R';
pub const SCMD_KIN_PID_DOWNSAMPLE: char = 'N';

pub const SCMD_KIN_CALIBRATION: char = 'C';
pub const SCMD_KIN_FLASH_STATUS: char = 'F';

/// Errors returned when parsing kinematics ASCII commands.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum KinematicsCommandError {
    /// Command string was empty.
    EmptyCommand,
    /// Unknown or missing subcommand character.
    UnknownSubcommand(char),
    /// Invalid or out-of-bounds motor/joint index.
    InvalidIndex(usize),
    /// Numerical parsing failed for argument.
    InvalidValue,
    /// Invalid sensor channel mapping syntax.
    InvalidMapping,
}

/// Robotic kinematics configuration and state representation.
#[derive(Clone, Debug, PartialEq)]
pub struct KinematicsConfig {
    /// Topology preset mode (0 = 4x 1-DOF, 1 = 2x 2-DOF, 2 = 1x 1-DOF + 1x 2-DOF, 3 = 2x 1-DOF + 1x 2-DOF, 255 = Custom).
    pub topology_mode: u8,
    /// Sensor channel routing for motors 0..3.
    pub sensor_map: [u8; 4],
    /// 2-DOF differential coupling bitmask.
    pub coupling_mask: u8,
    /// Outer joint target angles in radians for joints 0..3.
    pub joint_targets: [f32; 4],
    /// Measured joint feedback angles in radians for joints 0..3.
    pub joint_angles: [f32; 4],
    /// Measured joint feedback velocities in rad/s for joints 0..3.
    pub joint_velocities: [f32; 4],
    /// Transmission gear reduction ratios for joints 0..3.
    pub gear_ratio: [f32; 4],
    /// Transmission backlash deadband amplitude in radians for joints 0..3.
    pub backlash: [f32; 4],
    /// Transmission velocity smoothing threshold in rad/s for joints 0..3.
    pub omega_thresh: [f32; 4],
    /// Transmission torsional stiffness in Nm/rad for joints 0..3.
    pub stiffness: [f32; 4],
    /// Transmission joint zero offsets in motor radians for joints 0..3.
    pub zero_offset: [f32; 4],
    /// Outer position loop PID proportional gains for joints 0..3.
    pub pid_p: [f32; 4],
    /// Outer position loop PID integral gains for joints 0..3.
    pub pid_i: [f32; 4],
    /// Outer position loop PID derivative gains for joints 0..3.
    pub pid_d: [f32; 4],
    /// Outer position loop PID output limits for joints 0..3.
    pub pid_limit: [f32; 4],
    /// Outer position loop PID output ramp limits for joints 0..3.
    pub pid_ramp: [f32; 4],
    /// Outer position loop decimation factors relative to inner FOC steps for joints 0..3.
    pub pid_downsample: [u32; 4],
    /// Last commanded calibration trigger.
    pub calib_cmd: u8,
    /// Active calibration state (0 = Idle, 1 = Running, 2 = Done, 3 = Error).
    pub calib_state: u8,
    /// Non-volatile flash configuration validity status (1 = Valid, 0 = Uninitialized / Corrupted).
    pub flash_status: u8,
}

impl Default for KinematicsConfig {
    fn default() -> Self {
        Self {
            topology_mode: 0,
            sensor_map: [0, 1, 2, 3],
            coupling_mask: 0,
            joint_targets: [0.0; 4],
            joint_angles: [0.0; 4],
            joint_velocities: [0.0; 4],
            gear_ratio: [1.0; 4],
            backlash: [0.0; 4],
            omega_thresh: [0.05; 4],
            stiffness: [0.0; 4],
            zero_offset: [0.0; 4],
            pid_p: [0.0; 4],
            pid_i: [0.0; 4],
            pid_d: [0.0; 4],
            pid_limit: [0.0; 4],
            pid_ramp: [0.0; 4],
            pid_downsample: [1; 4],
            calib_cmd: 0,
            calib_state: 0,
            flash_status: 0,
        }
    }
}

impl KinematicsConfig {
    /// Create a new kinematics configuration with default parameters.
    pub fn new() -> Self {
        Self::default()
    }

    /// Parse and execute a command directly on this configuration.
    pub fn handle(&mut self, user_cmd: &str) -> Result<f32, KinematicsCommandError> {
        KinematicsCommander::handle(self, user_cmd)
    }

    /// Parse and execute a command, writing formatted responses to `writer`.
    #[cfg(feature = "embedded-io")]
    pub fn handle_with_writer<W: embedded_io::Write>(
        &mut self,
        user_cmd: &str,
        writer: &mut W,
        verbose: VerboseMode,
        decimals: u8,
    ) -> Result<(), W::Error> {
        KinematicsCommander::handle_with_writer(self, user_cmd, writer, verbose, decimals)
    }
}

/// ASCII parser and execution dispatcher for kinematics commands.
pub struct KinematicsCommander;

impl KinematicsCommander {
    /// Parse and execute a kinematics command string against `KinematicsConfig`.
    ///
    /// Accepts commands with or without the leading `'K'` command root identifier
    /// (e.g. `"KT 0 1.57"`, `"T 0 1.57"`, `"KA 0"`, `"A 0"`).
    pub fn handle(
        config: &mut KinematicsConfig,
        user_cmd: &str,
    ) -> Result<f32, KinematicsCommandError> {
        let mut trimmed = user_cmd.trim();
        if trimmed.is_empty() {
            return Err(KinematicsCommandError::EmptyCommand);
        }

        // Strip leading 'K' / 'k' if present
        if trimmed.starts_with('K') || trimmed.starts_with('k') {
            let after_k = trimmed[1..].trim_start();
            if after_k.is_empty() {
                // Bare "K" query: return topology mode or 0.0
                return Ok(config.topology_mode as f32);
            }
            trimmed = after_k;
            // If after_k also starts with 'K' and has another char (e.g. "K KT" -> "KT"), strip redundant 'K'
            if (trimmed.starts_with('K') || trimmed.starts_with('k')) && trimmed.len() > 1 {
                let second = trimmed.chars().nth(1).unwrap();
                if second.is_ascii_alphabetic() {
                    trimmed = &trimmed[1..];
                }
            }
        }

        let mut chars = trimmed.chars();
        let sub = chars
            .next()
            .ok_or(KinematicsCommandError::EmptyCommand)?
            .to_ascii_uppercase();
        let rest = chars.as_str();

        match sub {
            SCMD_KIN_TOPOLOGY => {
                // KM <mode> or KM
                let payload = rest.trim();
                if payload.is_empty() {
                    Ok(config.topology_mode as f32)
                } else {
                    let mode = if payload.starts_with("0x") || payload.starts_with("0X") {
                        u8::from_str_radix(&payload[2..], 16)
                            .map_err(|_| KinematicsCommandError::InvalidValue)?
                    } else {
                        payload
                            .parse::<u8>()
                            .map_err(|_| KinematicsCommandError::InvalidValue)?
                    };
                    config.topology_mode = mode;
                    Ok(config.topology_mode as f32)
                }
            }

            SCMD_KIN_SENSOR_MAP => {
                // KS <m>:<ch> or KS <m> <ch> or KS <m> or KS
                let payload = rest.trim();
                if payload.is_empty() {
                    return Ok(config.sensor_map[0] as f32);
                }
                if let Some((mot_str, ch_str)) = payload.split_once(':') {
                    let m = mot_str
                        .trim()
                        .parse::<usize>()
                        .map_err(|_| KinematicsCommandError::InvalidValue)?;
                    if m >= 4 {
                        return Err(KinematicsCommandError::InvalidIndex(m));
                    }
                    let ch = ch_str
                        .trim()
                        .parse::<u8>()
                        .map_err(|_| KinematicsCommandError::InvalidValue)?;
                    config.sensor_map[m] = ch;
                    Ok(config.sensor_map[m] as f32)
                } else {
                    let mut tokens = payload.split_ascii_whitespace();
                    let tok1 = tokens
                        .next()
                        .ok_or(KinematicsCommandError::InvalidMapping)?;
                    let m = tok1
                        .parse::<usize>()
                        .map_err(|_| KinematicsCommandError::InvalidValue)?;
                    if m >= 4 {
                        return Err(KinematicsCommandError::InvalidIndex(m));
                    }
                    if let Some(tok2) = tokens.next() {
                        let ch = tok2
                            .parse::<u8>()
                            .map_err(|_| KinematicsCommandError::InvalidValue)?;
                        config.sensor_map[m] = ch;
                        Ok(config.sensor_map[m] as f32)
                    } else {
                        // Query channel for motor m
                        Ok(config.sensor_map[m] as f32)
                    }
                }
            }

            SCMD_KIN_COUPLING_MASK => {
                // KX <mask> or KX
                let payload = rest.trim();
                if payload.is_empty() {
                    Ok(config.coupling_mask as f32)
                } else {
                    let mask = if payload.starts_with("0x") || payload.starts_with("0X") {
                        u8::from_str_radix(&payload[2..], 16)
                            .map_err(|_| KinematicsCommandError::InvalidValue)?
                    } else {
                        payload
                            .parse::<u8>()
                            .map_err(|_| KinematicsCommandError::InvalidValue)?
                    };
                    config.coupling_mask = mask;
                    Ok(config.coupling_mask as f32)
                }
            }

            SCMD_KIN_JOINT_TARGET => {
                // KT <j> <rad> or KT <j> or KT <rad> or KT
                let payload = rest.trim();
                if payload.is_empty() {
                    return Ok(config.joint_targets[0]);
                }
                let mut tokens = payload.split_ascii_whitespace();
                let tok1 = tokens.next().unwrap();
                if let Some(tok2) = tokens.next() {
                    let j = tok1
                        .parse::<usize>()
                        .map_err(|_| KinematicsCommandError::InvalidValue)?;
                    if j >= 4 {
                        return Err(KinematicsCommandError::InvalidIndex(j));
                    }
                    let rad = tok2
                        .parse::<f32>()
                        .map_err(|_| KinematicsCommandError::InvalidValue)?;
                    config.joint_targets[j] = rad;
                    Ok(config.joint_targets[j])
                } else {
                    if let Ok(j) = tok1.parse::<usize>() {
                        if j < 4 && !tok1.contains('.') {
                            return Ok(config.joint_targets[j]);
                        }
                    }
                    let rad = tok1
                        .parse::<f32>()
                        .map_err(|_| KinematicsCommandError::InvalidValue)?;
                    config.joint_targets[0] = rad;
                    Ok(config.joint_targets[0])
                }
            }

            SCMD_KIN_JOINT_ANGLE => {
                // KA <j> or KA
                let payload = rest.trim();
                let j = if payload.is_empty() {
                    0
                } else {
                    let mut tokens = payload.split_ascii_whitespace();
                    let tok1 = tokens.next().unwrap();
                    let idx = tok1
                        .parse::<usize>()
                        .map_err(|_| KinematicsCommandError::InvalidValue)?;
                    if idx >= 4 {
                        return Err(KinematicsCommandError::InvalidIndex(idx));
                    }
                    if let Some(tok2) = tokens.next() {
                        let rad = tok2
                            .parse::<f32>()
                            .map_err(|_| KinematicsCommandError::InvalidValue)?;
                        config.joint_angles[idx] = rad;
                    }
                    idx
                };
                Ok(config.joint_angles[j])
            }

            SCMD_KIN_JOINT_VELOCITY => {
                // KV <j> or KV
                let payload = rest.trim();
                let j = if payload.is_empty() {
                    0
                } else {
                    let mut tokens = payload.split_ascii_whitespace();
                    let tok1 = tokens.next().unwrap();
                    let idx = tok1
                        .parse::<usize>()
                        .map_err(|_| KinematicsCommandError::InvalidValue)?;
                    if idx >= 4 {
                        return Err(KinematicsCommandError::InvalidIndex(idx));
                    }
                    if let Some(tok2) = tokens.next() {
                        let vel = tok2
                            .parse::<f32>()
                            .map_err(|_| KinematicsCommandError::InvalidValue)?;
                        config.joint_velocities[idx] = vel;
                    }
                    idx
                };
                Ok(config.joint_velocities[j])
            }

            SCMD_KIN_GEAR_RATIO => Self::handle_joint_param(&mut config.gear_ratio, rest),
            SCMD_KIN_BACKLASH => Self::handle_joint_param(&mut config.backlash, rest),
            SCMD_KIN_OMEGA_THRESH => Self::handle_joint_param(&mut config.omega_thresh, rest),
            SCMD_KIN_STIFFNESS => Self::handle_joint_param(&mut config.stiffness, rest),
            SCMD_KIN_ZERO_OFFSET => Self::handle_joint_param(&mut config.zero_offset, rest),

            SCMD_KIN_PID_P => Self::handle_joint_param(&mut config.pid_p, rest),
            SCMD_KIN_PID_I => Self::handle_joint_param(&mut config.pid_i, rest),
            SCMD_KIN_PID_D => Self::handle_joint_param(&mut config.pid_d, rest),
            SCMD_KIN_PID_LIM => Self::handle_joint_param(&mut config.pid_limit, rest),
            SCMD_KIN_PID_RAMP => Self::handle_joint_param(&mut config.pid_ramp, rest),

            SCMD_KIN_PID_DOWNSAMPLE => {
                // KN <j> <int> or KN <j> or KN <int> or KN
                let trimmed_rest = rest.trim();
                if trimmed_rest.is_empty() {
                    return Ok(config.pid_downsample[0] as f32);
                }
                let mut tokens = trimmed_rest.split_ascii_whitespace();
                let tok1 = tokens.next().unwrap();
                if let Some(tok2) = tokens.next() {
                    let j = tok1
                        .parse::<usize>()
                        .map_err(|_| KinematicsCommandError::InvalidValue)?;
                    if j >= 4 {
                        return Err(KinematicsCommandError::InvalidIndex(j));
                    }
                    let val = tok2
                        .parse::<u32>()
                        .map_err(|_| KinematicsCommandError::InvalidValue)?;
                    config.pid_downsample[j] = if val == 0 { 1 } else { val };
                    Ok(config.pid_downsample[j] as f32)
                } else {
                    if let Ok(j) = tok1.parse::<usize>() {
                        if j < 4 && !tok1.contains('.') {
                            return Ok(config.pid_downsample[j] as f32);
                        }
                    }
                    let val = tok1
                        .parse::<u32>()
                        .map_err(|_| KinematicsCommandError::InvalidValue)?;
                    config.pid_downsample[0] = if val == 0 { 1 } else { val };
                    Ok(config.pid_downsample[0] as f32)
                }
            }

            SCMD_KIN_CALIBRATION => {
                // KC <cmd> or KC
                let payload = rest.trim();
                if payload.is_empty() {
                    Ok(config.calib_state as f32)
                } else {
                    let cmd = payload
                        .parse::<u8>()
                        .map_err(|_| KinematicsCommandError::InvalidValue)?;
                    config.calib_cmd = cmd;
                    match cmd {
                        1 => {
                            // Stall sweep / Start Autonomous
                            config.calib_state = 1;
                        }
                        2 => {
                            // Abort
                            config.calib_state = 0;
                        }
                        3 => {
                            // Save Flash
                            config.flash_status = 1;
                            config.calib_state = 2;
                        }
                        4 => {
                            // Load Flash
                            config.flash_status = 1;
                            config.calib_state = 0;
                        }
                        8 => {
                            // Reset
                            config.calib_state = 0;
                        }
                        _ => {}
                    }
                    Ok(config.calib_state as f32)
                }
            }

            SCMD_KIN_FLASH_STATUS => {
                // KF <int> or KF
                let payload = rest.trim();
                if payload.is_empty() {
                    Ok(config.flash_status as f32)
                } else {
                    let val = payload
                        .parse::<u8>()
                        .map_err(|_| KinematicsCommandError::InvalidValue)?;
                    config.flash_status = val;
                    Ok(config.flash_status as f32)
                }
            }

            other => Err(KinematicsCommandError::UnknownSubcommand(other)),
        }
    }

    fn handle_joint_param(
        arr: &mut [f32; 4],
        payload: &str,
    ) -> Result<f32, KinematicsCommandError> {
        let trimmed = payload.trim();
        if trimmed.is_empty() {
            return Ok(arr[0]);
        }
        let mut tokens = trimmed.split_ascii_whitespace();
        let tok1 = tokens.next().unwrap();
        if let Some(tok2) = tokens.next() {
            let j = tok1
                .parse::<usize>()
                .map_err(|_| KinematicsCommandError::InvalidValue)?;
            if j >= 4 {
                return Err(KinematicsCommandError::InvalidIndex(j));
            }
            let val = tok2
                .parse::<f32>()
                .map_err(|_| KinematicsCommandError::InvalidValue)?;
            arr[j] = val;
            Ok(arr[j])
        } else {
            if let Ok(j) = tok1.parse::<usize>() {
                if j < 4 && !tok1.contains('.') {
                    return Ok(arr[j]);
                }
            }
            let val = tok1
                .parse::<f32>()
                .map_err(|_| KinematicsCommandError::InvalidValue)?;
            arr[0] = val;
            Ok(arr[0])
        }
    }

    /// Helper to extract the single-character subcommand for formatted output.
    pub fn extract_subcommand(user_cmd: &str) -> Option<char> {
        let mut trimmed = user_cmd.trim();
        if trimmed.is_empty() {
            return None;
        }
        if trimmed.starts_with('K') || trimmed.starts_with('k') {
            let after_k = trimmed[1..].trim_start();
            if after_k.is_empty() {
                return Some('K');
            }
            trimmed = after_k;
            if (trimmed.starts_with('K') || trimmed.starts_with('k')) && trimmed.len() > 1 {
                let second = trimmed.chars().nth(1).unwrap();
                if second.is_ascii_alphabetic() {
                    trimmed = &trimmed[1..];
                }
            }
        }
        trimmed.chars().next()
    }

    /// Parse and execute a command, writing formatted responses to `writer`.
    #[cfg(feature = "embedded-io")]
    pub fn handle_with_writer<W: embedded_io::Write>(
        config: &mut KinematicsConfig,
        user_cmd: &str,
        writer: &mut W,
        verbose: VerboseMode,
        decimals: u8,
    ) -> Result<(), W::Error> {
        let trimmed = user_cmd.trim();
        if trimmed.is_empty() {
            return Self::write_error(writer, verbose);
        }

        let sub = Self::extract_subcommand(trimmed);

        match Self::handle(config, trimmed) {
            Ok(val) => {
                if verbose == VerboseMode::nothing {
                    return Ok(());
                }

                if verbose == VerboseMode::machine_readable {
                    writer.write_all(b"K")?;
                    if let Some(sc) = sub {
                        let mut b = [0u8; 4];
                        writer.write_all(sc.to_ascii_uppercase().encode_utf8(&mut b).as_bytes())?;
                    }
                } else if verbose == VerboseMode::user_friendly {
                    if let Some(sc) = sub {
                        writer.write_all(b"k")?;
                        let mut b = [0u8; 4];
                        writer.write_all(sc.to_ascii_lowercase().encode_utf8(&mut b).as_bytes())?;
                        writer.write_all(b": ")?;
                    }
                }

                Self::write_float(writer, val, decimals)?;
                writer.write_all(b"\r\n")?;
            }
            Err(_) => {
                Self::write_error(writer, verbose)?;
            }
        }
        Ok(())
    }

    #[cfg(feature = "embedded-io")]
    fn write_error<W: embedded_io::Write>(
        writer: &mut W,
        verbose: VerboseMode,
    ) -> Result<(), W::Error> {
        if verbose != VerboseMode::nothing {
            writer.write_all(b"err\r\n")?;
        }
        Ok(())
    }

    #[cfg(feature = "embedded-io")]
    fn write_float<W: embedded_io::Write>(
        writer: &mut W,
        val: f32,
        decimals: u8,
    ) -> Result<(), W::Error> {
        struct IoAdapter<'a, W: embedded_io::Write> {
            inner: &'a mut W,
            err: Option<W::Error>,
        }
        impl<'a, W: embedded_io::Write> core::fmt::Write for IoAdapter<'a, W> {
            fn write_str(&mut self, s: &str) -> core::fmt::Result {
                match self.inner.write_all(s.as_bytes()) {
                    Ok(()) => Ok(()),
                    Err(e) => {
                        self.err = Some(e);
                        Err(core::fmt::Error)
                    }
                }
            }
        }

        let mut adapter = IoAdapter {
            inner: writer,
            err: None,
        };
        let prec = (decimals as usize).min(8);
        let _ = core::fmt::write(&mut adapter, format_args!("{:.prec$}", val, prec = prec));
        if let Some(err) = adapter.err {
            return Err(err);
        }
        Ok(())
    }
}

/// Four-axis multi-motor and kinematics command dispatcher helper.
///
/// Registers and coordinates:
/// - `'A'` : Motor A control and configuration
/// - `'B'` : Motor B control and configuration
/// - `'C'` : Motor C control and configuration
/// - `'D'` : Motor D control and configuration
/// - `'K'` : Robotics kinematics, transmissions, and topology routing
#[cfg(feature = "barefoc")]
pub struct Commander4Axis<C: barefoc::common::time::Clock = barefoc::motor::base::NullClock> {
    /// Underlying generic commander for scans, verbose configuration, and custom callbacks.
    pub commander: crate::commander::Commander<20>,
    /// Axis A motor configuration.
    pub motor_a: barefoc::motor::base::MotorConfig<C>,
    /// Axis B motor configuration.
    pub motor_b: barefoc::motor::base::MotorConfig<C>,
    /// Axis C motor configuration.
    pub motor_c: barefoc::motor::base::MotorConfig<C>,
    /// Axis D motor configuration.
    pub motor_d: barefoc::motor::base::MotorConfig<C>,
    /// Multi-axis kinematics configuration.
    pub kinematics: KinematicsConfig,
}

#[cfg(feature = "barefoc")]
impl Commander4Axis<barefoc::motor::base::NullClock> {
    /// Create a new 4-axis commander with standard default configuration and null clock.
    pub fn new() -> Self {
        let clock = barefoc::motor::base::NullClock;
        Self::new_with_clock(&clock)
    }
}

#[cfg(feature = "barefoc")]
impl Default for Commander4Axis<barefoc::motor::base::NullClock> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(feature = "barefoc")]
impl<C: barefoc::common::time::Clock> Commander4Axis<C> {
    /// Create a new 4-axis commander with clock reference.
    pub fn new_with_clock(clock: &C) -> Self {
        Self {
            commander: crate::commander::Commander::new(),
            motor_a: barefoc::motor::base::MotorConfig::new(clock),
            motor_b: barefoc::motor::base::MotorConfig::new(clock),
            motor_c: barefoc::motor::base::MotorConfig::new(clock),
            motor_d: barefoc::motor::base::MotorConfig::new(clock),
            kinematics: KinematicsConfig::default(),
        }
    }

    /// Builder method configuring custom motors.
    pub fn with_motors(
        mut self,
        a: barefoc::motor::base::MotorConfig<C>,
        b: barefoc::motor::base::MotorConfig<C>,
        c: barefoc::motor::base::MotorConfig<C>,
        d: barefoc::motor::base::MotorConfig<C>,
    ) -> Self {
        self.motor_a = a;
        self.motor_b = b;
        self.motor_c = c;
        self.motor_d = d;
        self
    }

    /// Builder method configuring kinematics.
    pub fn with_kinematics(mut self, kinematics: KinematicsConfig) -> Self {
        self.kinematics = kinematics;
        self
    }

    /// Parse and execute a command string across motors A..D or kinematics K.
    pub fn run(&mut self, user_input: &str) {
        let input = user_input.trim_matches(|c| c == '\r' || c == '\n');
        if input.is_empty() {
            return;
        }

        let mut chars = input.chars();
        let id = match chars.next() {
            Some(c) => c,
            None => return,
        };
        let payload = chars.as_str().trim_start();

        match id {
            'A' => {
                let _ = crate::motor::MotorCommander::handle(&mut self.motor_a, payload);
            }
            'B' => {
                let _ = crate::motor::MotorCommander::handle(&mut self.motor_b, payload);
            }
            'C' => {
                let _ = crate::motor::MotorCommander::handle(&mut self.motor_c, payload);
            }
            'D' => {
                let _ = crate::motor::MotorCommander::handle(&mut self.motor_d, payload);
            }
            'K' => {
                let _ = KinematicsCommander::handle(&mut self.kinematics, payload);
            }
            '?' | '@' | '#' => {
                self.commander.run(user_input);
            }
            _ => {
                self.commander.run(user_input);
            }
        }
    }

    /// Parse and execute a command string, writing human/machine-readable responses to `writer`.
    #[cfg(feature = "embedded-io")]
    pub fn run_with_writer<W: embedded_io::Write>(
        &mut self,
        user_input: &str,
        writer: &mut W,
    ) -> Result<(), W::Error> {
        let input = user_input.trim_matches(|c| c == '\r' || c == '\n');
        if input.is_empty() {
            return Ok(());
        }

        let mut chars = input.chars();
        let id = match chars.next() {
            Some(c) => c,
            None => return Ok(()),
        };
        let payload = chars.as_str().trim_start();
        let verbose = self.commander.verbose;
        let decimals = self.commander.decimal_places;

        match id {
            'A' => {
                crate::motor::MotorCommander::handle_with_writer(
                    &mut self.motor_a,
                    payload,
                    writer,
                    verbose,
                    decimals,
                )
            }
            'B' => {
                crate::motor::MotorCommander::handle_with_writer(
                    &mut self.motor_b,
                    payload,
                    writer,
                    verbose,
                    decimals,
                )
            }
            'C' => {
                crate::motor::MotorCommander::handle_with_writer(
                    &mut self.motor_c,
                    payload,
                    writer,
                    verbose,
                    decimals,
                )
            }
            'D' => {
                crate::motor::MotorCommander::handle_with_writer(
                    &mut self.motor_d,
                    payload,
                    writer,
                    verbose,
                    decimals,
                )
            }
            'K' => {
                KinematicsCommander::handle_with_writer(
                    &mut self.kinematics,
                    payload,
                    writer,
                    verbose,
                    decimals,
                )
            }
            '?' => {
                if verbose == VerboseMode::machine_readable {
                    writer.write_all(b"?A:Motor A\r\n?B:Motor B\r\n?C:Motor C\r\nD:Motor D\r\n?K:Kinematics\r\n")?;
                } else {
                    writer.write_all(b"A:Motor A\r\nB:Motor B\r\nC:Motor C\r\nD:Motor D\r\nK:Kinematics\r\n")?;
                }
                self.commander.run_with_writer(user_input, writer)
            }
            '@' | '#' => self.commander.run_with_writer(user_input, writer),
            _ => self.commander.run_with_writer(user_input, writer),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_topology_and_sensor_mapping() {
        let mut config = KinematicsConfig::new();

        // Topology: KM 2
        KinematicsCommander::handle(&mut config, "KM 2").unwrap();
        assert_eq!(config.topology_mode, 2);
        let res = KinematicsCommander::handle(&mut config, "KM").unwrap();
        assert_eq!(res, 2.0);

        // Hex topology: KM 0x01
        KinematicsCommander::handle(&mut config, "KM 0x01").unwrap();
        assert_eq!(config.topology_mode, 1);

        // Sensor mapping: KS 0:3
        KinematicsCommander::handle(&mut config, "KS 0:3").unwrap();
        assert_eq!(config.sensor_map[0], 3);

        // Sensor mapping space separated: KS 1 2
        KinematicsCommander::handle(&mut config, "KS 1 2").unwrap();
        assert_eq!(config.sensor_map[1], 2);

        // Query sensor mapping
        assert_eq!(KinematicsCommander::handle(&mut config, "KS 0").unwrap(), 3.0);
        assert_eq!(KinematicsCommander::handle(&mut config, "KS 1").unwrap(), 2.0);

        // Coupling mask: KX 0x05
        KinematicsCommander::handle(&mut config, "KX 0x05").unwrap();
        assert_eq!(config.coupling_mask, 5);
        assert_eq!(KinematicsCommander::handle(&mut config, "KX").unwrap(), 5.0);
    }

    #[test]
    fn test_joint_targets_and_telemetry() {
        let mut config = KinematicsConfig::new();

        // Joint target: KT 0 1.57
        KinematicsCommander::handle(&mut config, "KT 0 1.57").unwrap();
        assert_eq!(config.joint_targets[0], 1.57);

        // Joint target joint 1: KT 1 -0.785
        KinematicsCommander::handle(&mut config, "KT 1 -0.785").unwrap();
        assert_eq!(config.joint_targets[1], -0.785);

        // Query target
        assert_eq!(KinematicsCommander::handle(&mut config, "KT 0").unwrap(), 1.57);
        assert_eq!(KinematicsCommander::handle(&mut config, "KT 1").unwrap(), -0.785);

        // Feedback angle & velocity setting/querying
        KinematicsCommander::handle(&mut config, "KA 0 1.57").unwrap();
        assert_eq!(config.joint_angles[0], 1.57);
        assert_eq!(KinematicsCommander::handle(&mut config, "KA 0").unwrap(), 1.57);

        KinematicsCommander::handle(&mut config, "KV 0 10.5").unwrap();
        assert_eq!(config.joint_velocities[0], 10.5);
        assert_eq!(KinematicsCommander::handle(&mut config, "KV 0").unwrap(), 10.5);
    }

    #[test]
    fn test_transmission_and_pid_parameters() {
        let mut config = KinematicsConfig::new();

        // Gear ratio: KG 0 50.0
        KinematicsCommander::handle(&mut config, "KG 0 50.0").unwrap();
        assert_eq!(config.gear_ratio[0], 50.0);
        assert_eq!(KinematicsCommander::handle(&mut config, "KG 0").unwrap(), 50.0);

        // Backlash: KB 0 0.02
        KinematicsCommander::handle(&mut config, "KB 0 0.02").unwrap();
        assert_eq!(config.backlash[0], 0.02);

        // Omega thresh: KW 0 0.1
        KinematicsCommander::handle(&mut config, "KW 0 0.1").unwrap();
        assert_eq!(config.omega_thresh[0], 0.1);

        // Stiffness: KK 0 250.0
        KinematicsCommander::handle(&mut config, "KK 0 250.0").unwrap();
        assert_eq!(config.stiffness[0], 250.0);

        // Zero offset: KZ 0 0.35
        KinematicsCommander::handle(&mut config, "KZ 0 0.35").unwrap();
        assert_eq!(config.zero_offset[0], 0.35);

        // PID parameters: KP, KI, KD, KL, KR, KN
        KinematicsCommander::handle(&mut config, "KP 0 4.5").unwrap();
        assert_eq!(config.pid_p[0], 4.5);

        KinematicsCommander::handle(&mut config, "KI 0 0.8").unwrap();
        assert_eq!(config.pid_i[0], 0.8);

        KinematicsCommander::handle(&mut config, "KD 0 0.05").unwrap();
        assert_eq!(config.pid_d[0], 0.05);

        KinematicsCommander::handle(&mut config, "KL 0 24.0").unwrap();
        assert_eq!(config.pid_limit[0], 24.0);

        KinematicsCommander::handle(&mut config, "KR 0 100.0").unwrap();
        assert_eq!(config.pid_ramp[0], 100.0);

        KinematicsCommander::handle(&mut config, "KN 0 4").unwrap();
        assert_eq!(config.pid_downsample[0], 4);
    }

    #[test]
    fn test_calibration_and_flash() {
        let mut config = KinematicsConfig::new();

        // Calibration Stall Sweep: KC 1 -> state 1 (Running)
        let state = KinematicsCommander::handle(&mut config, "KC 1").unwrap();
        assert_eq!(state, 1.0);
        assert_eq!(config.calib_state, 1);

        // Abort: KC 2 -> state 0 (Idle)
        KinematicsCommander::handle(&mut config, "KC 2").unwrap();
        assert_eq!(config.calib_state, 0);

        // Save Flash: KC 3 -> state 2 (Done), flash 1
        KinematicsCommander::handle(&mut config, "KC 3").unwrap();
        assert_eq!(config.calib_state, 2);
        assert_eq!(config.flash_status, 1);

        // Flash query: KF
        assert_eq!(KinematicsCommander::handle(&mut config, "KF").unwrap(), 1.0);
    }

    #[test]
    #[cfg(feature = "embedded-io")]
    fn test_writer_verbose_modes() {
        let mut config = KinematicsConfig::new();
        config.joint_angles[0] = 1.57;

        // User-friendly mode
        let mut out = [0u8; 64];
        let mut cursor = &mut out[..];
        KinematicsCommander::handle_with_writer(
            &mut config,
            "KA 0",
            &mut cursor,
            VerboseMode::user_friendly,
            4,
        )
        .unwrap();

        let full_slice = &out[..out.iter().position(|&b| b == 0).unwrap_or(out.len())];
        let text = core::str::from_utf8(full_slice).unwrap();
        assert!(text.starts_with("ka: 1.5700"));

        // Machine readable mode
        let mut out2 = [0u8; 64];
        let mut cursor2 = &mut out2[..];
        KinematicsCommander::handle_with_writer(
            &mut config,
            "KA 0",
            &mut cursor2,
            VerboseMode::machine_readable,
            4,
        )
        .unwrap();
        let full_slice2 = &out2[..out2.iter().position(|&b| b == 0).unwrap_or(out2.len())];
        let text2 = core::str::from_utf8(full_slice2).unwrap();
        assert!(text2.starts_with("KA1.5700"));
    }

    #[test]
    #[cfg(feature = "barefoc")]
    fn test_commander_4axis_dispatch() {
        let mut cmd4 = Commander4Axis::new();

        // Multi-axis motor dispatch: A 10.0, B 5.0
        cmd4.run("A 10.0\n");
        cmd4.run("B 5.0\n");

        assert_eq!(cmd4.motor_a.target, 10.0);
        assert_eq!(cmd4.motor_b.target, 5.0);

        // Kinematics dispatch: KT 0 1.57
        cmd4.run("KT 0 1.57\n");
        assert_eq!(cmd4.kinematics.joint_targets[0], 1.57);

        // Motor C and D
        cmd4.run("C 2.5\n");
        cmd4.run("D -1.2\n");
        assert_eq!(cmd4.motor_c.target, 2.5);
        assert_eq!(cmd4.motor_d.target, -1.2);
    }

    #[test]
    #[cfg(all(feature = "barefoc", feature = "embedded-io"))]
    fn test_spec_scenarios() {
        let mut cmd4 = Commander4Axis::new();
        cmd4.commander.decimal_places = 4;

        // Scenario: Setting joint target via ASCII
        // WHEN user sends KT 0 1.57\n
        // THEN joint 0's outer position loop target is updated to 1.57 radians.
        let mut buf = [0u8; 64];
        let mut cursor = &mut buf[..];
        cmd4.run_with_writer("KT 0 1.57\n", &mut cursor).unwrap();
        assert_eq!(cmd4.kinematics.joint_targets[0], 1.57);

        // Scenario: Reading joint angle via ASCII
        // WHEN user sends KA 0\n
        // THEN Commander returns ka: 1.5700\n formatted per verbose mode.
        cmd4.kinematics.joint_angles[0] = 1.57;
        let mut out = [0u8; 64];
        let mut cursor2 = &mut out[..];
        cmd4.run_with_writer("KA 0\n", &mut cursor2).unwrap();
        let full = &out[..out.iter().position(|&b| b == 0).unwrap_or(out.len())];
        let reply = core::str::from_utf8(full).unwrap();
        assert_eq!(reply, "ka: 1.5700\r\n");

        // Scenario: Multi-axis dispatch
        // WHEN user sends A 10.0\n followed by B 5.0\n
        // THEN Motor A targets 10.0 rad/s and Motor B targets 5.0 rad/s independently.
        cmd4.run("A 10.0\n");
        cmd4.run("B 5.0\n");
        assert_eq!(cmd4.motor_a.target, 10.0);
        assert_eq!(cmd4.motor_b.target, 5.0);
    }

    #[test]
    fn test_error_handling() {
        let mut config = KinematicsConfig::new();

        // Empty command
        assert_eq!(
            KinematicsCommander::handle(&mut config, "").unwrap_err(),
            KinematicsCommandError::EmptyCommand
        );

        // Invalid value
        assert_eq!(
            KinematicsCommander::handle(&mut config, "KZ_invalid 0 1.0").unwrap_err(),
            KinematicsCommandError::InvalidValue
        );
        assert!(matches!(
            KinematicsCommander::handle(&mut config, "Q 1.0"),
            Err(KinematicsCommandError::UnknownSubcommand('Q'))
        ));

        // Invalid index
        assert_eq!(
            KinematicsCommander::handle(&mut config, "KT 99 1.57").unwrap_err(),
            KinematicsCommandError::InvalidIndex(99)
        );

        // Invalid mapping
        assert_eq!(
            KinematicsCommander::handle(&mut config, "KS 99:1").unwrap_err(),
            KinematicsCommandError::InvalidIndex(99)
        );
    }
}
