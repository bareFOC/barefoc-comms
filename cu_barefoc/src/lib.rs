//! # `cu-barefoc`
//!
//! A [Copper](https://github.com/copper-project/copper-rs) `CuBridge` for
//! multi-axis BareFOC actuator networks (FishyServo multi-axis DC joints and
//! Miniata BLDC smart servos).
//!
//! The bridge is host-side and therefore uses `std`, but the SDK it reuses —
//! [`barefoc_master`] — is `#![no_std]` and zero-heap. All setpoints are
//! dispatched in one atomic `INST_SYNC_WRITE` frame built by
//! [`barefoc_master::SyncGroup`], and telemetry is cached in fixed-capacity
//! messages so the hot path performs no allocation.
//!
//! ## Upstream contracts
//!
//! - `ref/copper-rs/core/cu29_runtime/src/cubridge.rs` — `CuBridge`, channel
//!   descriptors and lifecycle.
//! - `ref/copper-rs/core/cu29_runtime/src/payload.rs` — `CuArray`.
//! - `barefoc-master/src/{scanner,client,sync,registers}.rs` — discovery,
//!   control-table access, atomic dispatch and register map.
//! - `bareFOC/src/control_table.rs` — firmware-side register definitions.
//!
//! ## Transports
//!
//! [`ActuatorBus`] is the internal transport trait. Implementations:
//! [`MockBus`] (offline tests), [`I2cBus`] (`feature = "i2c"`),
//! [`SerialBus`] (`feature = "serial"`) and [`CanBus`] (`feature = "can"`).
//! Hardware-dependent adapters are feature gated; the bridge core is portable.

#![forbid(unsafe_code)]

pub mod bridge;
pub mod messages;
pub mod transport;

pub use bridge::{
    BarefocBridge, BarefocRxChannels, BarefocRxId, BarefocTxChannels, BarefocTxId, UnitMode,
};
pub use messages::{JointPositions, JointTelemetry, JointTorques, JointVelocities, MAX_ACTUATORS};
pub use transport::{ActuatorBus, MasterTransport};

#[cfg(feature = "i2c")]
pub use transport::I2cBus;
#[cfg(all(feature = "i2c", feature = "serial", target_os = "linux"))]
pub use transport::LinuxI2cBus;
#[cfg(feature = "can")]
pub use transport::{CanBus, CanFrame, CAN_FD_PAYLOAD_LEN};
#[cfg(feature = "serial")]
pub use transport::{LinuxSerialBus, SerialBus};
#[cfg(any(test, feature = "mock"))]
pub use transport::{MockBus, RegisterWrite};

use core::fmt;

use barefoc_master::client::ClientError;
use barefoc_master::sync::SyncError;
use barefoc_master::transport::TransportError;
use cu29_traits::CuError;

/// Legacy stage/MOSFET temperature register (`u16`, whole degrees Celsius).
///
/// Upstream: `bareFOC/src/control_table.rs` (`REG_PRESENT_TEMPERATURE`, `0x142`).
/// The extended MCU/motor zones live at `0x16A`/`0x16E`; this bridge reports
/// the canonical stage zone in `JointTelemetry::temperatures`.
pub const REG_PRESENT_TEMPERATURE: u16 = 0x142;

/// Errors produced by an [`ActuatorBus`] transaction.
///
/// Upstream: `barefoc-master/src/transport.rs` (`TransportError`),
/// `barefoc-comms/barefoc-multibus/src/client.rs` (`ClientError`).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum BusError {
    /// Bus/IO implementation reported a failure. The string is the concrete
    /// error type name (obtained without allocation via `type_name`).
    Io(&'static str),
    /// The peer did not answer within the transport timeout.
    Timeout,
    /// The peer did not acknowledge the transaction.
    NoAcknowledge,
    /// A staging buffer or caller output buffer was too small.
    BufferTooSmall,
    /// Node or register address not representable on the bus.
    InvalidAddress(u16),
    /// Malformed or unsupported protocol frame.
    Protocol(&'static str),
}

impl fmt::Display for BusError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io(name) => write!(f, "bus I/O error ({name})"),
            Self::Timeout => f.write_str("bus timeout"),
            Self::NoAcknowledge => f.write_str("no acknowledge from node"),
            Self::BufferTooSmall => f.write_str("bus buffer too small"),
            Self::InvalidAddress(addr) => write!(f, "invalid bus address 0x{addr:04X}"),
            Self::Protocol(msg) => write!(f, "bus protocol error: {msg}"),
        }
    }
}

impl std::error::Error for BusError {}

/// Typed errors returned by the bridge lifecycle instead of panicking.
///
/// Upstream: `ref/copper-rs/core/cu29_runtime/src/cubridge.rs` (`CuBridge`
/// lifecycle returns `CuResult`), `barefoc-master/src/sync.rs` (`SyncError`).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum BridgeError {
    /// Underlying transport failure.
    Bus(BusError),
    /// A message or target list exceeded the configured capacity.
    Capacity {
        /// Number of joints supplied by the caller.
        requested: usize,
        /// Maximum number of joints accepted by the bridge.
        maximum: usize,
    },
    /// A Copper channel carried no payload.
    MissingPayload(&'static str),
    /// Discovery found no responding BareFOC nodes.
    NoNodes,
    /// Configuration value was missing or invalid.
    Config(&'static str),
}

impl fmt::Display for BridgeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Bus(e) => write!(f, "BareFOC bridge bus error: {e}"),
            Self::Capacity { requested, maximum } => write!(
                f,
                "joint capacity exceeded: {requested} joints supplied, maximum is {maximum}"
            ),
            Self::MissingPayload(channel) => write!(f, "channel {channel} has no payload"),
            Self::NoNodes => f.write_str("no BareFOC nodes discovered on the bus"),
            Self::Config(key) => write!(f, "invalid or missing bridge config key: {key}"),
        }
    }
}

impl std::error::Error for BridgeError {}

impl From<BusError> for BridgeError {
    fn from(err: BusError) -> Self {
        Self::Bus(err)
    }
}

impl From<TransportError<BusError>> for BridgeError {
    fn from(err: TransportError<BusError>) -> Self {
        Self::Bus(match err {
            TransportError::Bus(e) => e,
            TransportError::Timeout => BusError::Timeout,
            TransportError::NoAcknowledge => BusError::NoAcknowledge,
            TransportError::BufferTooSmall => BusError::BufferTooSmall,
            TransportError::InvalidAddress(addr) => BusError::InvalidAddress(addr),
            TransportError::Protocol(msg) => BusError::Protocol(msg),
        })
    }
}

impl From<ClientError<BusError>> for BridgeError {
    fn from(err: ClientError<BusError>) -> Self {
        match err {
            ClientError::Transport(e) => Self::from(e),
            ClientError::Bounds(_) => {
                Self::Bus(BusError::Protocol("register access out of bounds"))
            }
        }
    }
}

impl From<SyncError> for BridgeError {
    fn from(err: SyncError) -> Self {
        Self::Bus(BusError::Protocol(match err {
            SyncError::Empty => "sync group is empty",
            SyncError::TooManyNodes => "too many nodes in sync group",
            SyncError::BufferTooSmall => "sync frame buffer too small",
            SyncError::LengthMismatch => "sync write target length mismatch",
        }))
    }
}

impl From<BridgeError> for CuError {
    fn from(err: BridgeError) -> Self {
        CuError::new_with_cause("BareFOC bridge failure", err)
    }
}

impl From<BusError> for CuError {
    fn from(err: BusError) -> Self {
        CuError::from(BridgeError::from(err))
    }
}
