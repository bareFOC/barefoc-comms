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
