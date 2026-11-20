//! # barefoc-commander
//!
//! Pure-Rust, `#![no_std]` implementation of the Arduino SimpleFOC
//! [`Commander`](https://docs.simplefoc.com/commander_interface) ASCII serial console
//! tuning and inspection protocol.
//!
//! ## Upstream Attribution
//!
//! Directly ported from and inspired by the SimpleFOC project created by
//! Antun Skuric and the SimpleFOC community.
//! - Upstream Reference: `source/Arduino-FOC/src/communication/Commander.h`
//! - Upstream Reference: `source/Arduino-FOC/src/communication/Commander.cpp`
//! - Upstream Reference: `source/Arduino-FOC/src/communication/commands.h`

#![no_std]

pub mod commander;
pub mod io;
pub mod kinematics;
#[cfg(feature = "barefoc")]
pub mod motor;
#[cfg(feature = "barefoc")]
pub mod servo;

pub use commander::{
    CallbackEntry, CommandCallback, Commander, VerboseMode, CMD_DECIMAL, CMD_SCAN, CMD_VERBOSE,
};
pub use io::{LineError, LineProcessor, StreamError};
pub use kinematics::{
    KinematicsCommandError, KinematicsCommander, KinematicsConfig, CMD_KINEMATICS,
    SCMD_KIN_BACKLASH, SCMD_KIN_CALIBRATION, SCMD_KIN_COUPLING_MASK, SCMD_KIN_FLASH_STATUS,
    SCMD_KIN_GEAR_RATIO, SCMD_KIN_JOINT_ANGLE, SCMD_KIN_JOINT_TARGET, SCMD_KIN_JOINT_VELOCITY,
    SCMD_KIN_OMEGA_THRESH, SCMD_KIN_PID_D, SCMD_KIN_PID_DOWNSAMPLE, SCMD_KIN_PID_I,
    SCMD_KIN_PID_LIM, SCMD_KIN_PID_P, SCMD_KIN_PID_RAMP, SCMD_KIN_SENSOR_MAP, SCMD_KIN_STIFFNESS,
    SCMD_KIN_TOPOLOGY, SCMD_KIN_ZERO_OFFSET,
};
#[cfg(feature = "barefoc")]
pub use kinematics::Commander4Axis;
#[cfg(feature = "barefoc")]
pub use motor::{MotorCommandError, MotorCommander};
#[cfg(feature = "barefoc")]
pub use servo::SmartServoCommander;

/// Lightweight ASCII string to f32 parser avoiding core::num::dec2flt's 10 KiB lookup table.
pub fn parse_f32(s: &str) -> Result<f32, ()> {
    let s = s.trim();
    if s.is_empty() {
        return Err(());
    }
    let (s, negative) = match s.strip_prefix('-') {
        Some(rest) => (rest, true),
        None => (s.strip_prefix('+').unwrap_or(s), false),
    };
    if s.is_empty() {
        return Err(());
    }
    let mut parts = s.split('.');
    let int_part = parts.next().unwrap();
    let frac_part = parts.next();
    if parts.next().is_some() {
        return Err(());
    }

    let mut integer_val: u64 = 0;
    if !int_part.is_empty() {
        for &b in int_part.as_bytes() {
            if !b.is_ascii_digit() {
                return Err(());
            }
            integer_val = integer_val
                .checked_mul(10)
                .and_then(|v| v.checked_add((b - b'0') as u64))
                .ok_or(())?;
        }
    } else if frac_part.is_none() {
        return Err(());
    }

    let mut frac_val: u64 = 0;
    let mut frac_div: f64 = 1.0;
    if let Some(frac) = frac_part {
        if frac.is_empty() && int_part.is_empty() {
            return Err(());
        }
        for &b in frac.as_bytes() {
            if !b.is_ascii_digit() {
                return Err(());
            }
            frac_val = frac_val
                .checked_mul(10)
                .and_then(|v| v.checked_add((b - b'0') as u64))
                .ok_or(())?;
            frac_div *= 10.0;
        }
    }

    let result = (integer_val as f64) + ((frac_val as f64) / frac_div);
    let result_f32 = result as f32;
    Ok(if negative { -result_f32 } else { result_f32 })
}

