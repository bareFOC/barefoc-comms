//! Fixed-capacity, reflectable Copper messages for BareFOC joint channels.
//!
//! Every message is backed by [`cu29::payload::CuArray`], a stack-allocated
//! fixed-capacity container that implements Copper's `Encode`, `Decode`,
//! `Reflect`, `Serialize` and `Deserialize` contracts. Encoding a message never
//! touches the heap, so these types can be recorded in `.copper` logs and
//! replayed into `barefoc-sim` deterministically.
//!
//! `Encode`/`Decode` are implemented by forwarding to `CuArray` rather than
//! derived: `CuArray` implements `Decode<()>` (not a generic `Decode<Context>`),
//! which the 1.3.x bincode derive cannot express. The wire format is identical
//! to a derived implementation.
//!
//! Upstream: `ref/copper-rs/core/cu29_runtime/src/payload.rs` (`CuArray`),
//! `ref/copper-rs/components/bridges/cu_feetech/src/messages.rs`
//! (`JointPositions` type alias), `ref/copper-rs/core/cu29_runtime/src/cubridge.rs`
//! (`CuMsgPayload` derives).

use cu29::bincode::de::Decoder;
use cu29::bincode::enc::Encoder;
use cu29::bincode::error::{DecodeError, EncodeError};
use cu29::bincode::{Decode, Encode};
use cu29::payload::CuArray;
use cu29::reflect::Reflect;
use serde::{Deserialize, Serialize};

use crate::BridgeError;

/// Maximum number of joints carried by one bridge message.
///
/// Matches `barefoc_master::scanner::MAX_NODES` (one setpoint per discovered
/// node) and keeps the largest message well below one cache line per channel.
pub const MAX_ACTUATORS: usize = 16;

/// Generate a distinct, strongly typed joint message with a fixed capacity.
macro_rules! joint_message {
    ($(#[$meta:meta])* $name:ident) => {
        $(#[$meta])*
        #[derive(Clone, Debug, Default, Reflect, Serialize, Deserialize)]
        pub struct $name {
            /// Joint values in the unit configured on the bridge.
            pub values: CuArray<f32, MAX_ACTUATORS>,
        }

        impl $name {
            /// Build a message from a slice, rejecting more than
            /// [`MAX_ACTUATORS`] joints with a typed [`BridgeError::Capacity`].
            pub fn from_slice(values: &[f32]) -> Result<Self, BridgeError> {
                if values.len() > MAX_ACTUATORS {
                    return Err(BridgeError::Capacity {
                        requested: values.len(),
                        maximum: MAX_ACTUATORS,
                    });
                }
                let mut message = Self::default();
                message.values.fill_from_iter(values.iter().copied());
                Ok(message)
            }

            /// Borrow the joint values as a slice.
            pub fn as_slice(&self) -> &[f32] {
                self.values.as_slice()
            }

            /// Number of populated joint slots.
            pub fn len(&self) -> usize {
                self.values.len()
            }

            /// Returns `true` when no joint values are populated.
            pub fn is_empty(&self) -> bool {
                self.values.is_empty()
            }
        }

        impl PartialEq for $name {
            fn eq(&self, other: &Self) -> bool {
                self.values.as_slice() == other.values.as_slice()
            }
        }

        impl Encode for $name {
            fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
                Encode::encode(&self.values, encoder)
            }
        }

        impl Decode<()> for $name {
            fn decode<D: Decoder<Context = ()>>(decoder: &mut D) -> Result<Self, DecodeError> {
                Ok(Self {
                    values: Decode::decode(decoder)?,
                })
            }
        }

    };
}

joint_message! {
    /// Desired joint angles, in the bridge's configured angular unit.
    ///
    /// Upstream: `barefoc-master/src/registers.rs` (`REG_GOAL_POSITION`, `0x130`).
    JointPositions
}

joint_message! {
    /// Desired joint velocities, in the bridge's configured angular unit per second.
    ///
    /// Upstream: `barefoc-master/src/registers.rs` (`REG_GOAL_VELOCITY`, `0x134`).
    JointVelocities
}

joint_message! {
    /// Desired torque-producing currents, in amperes.
    ///
    /// Upstream: `barefoc-master/src/registers.rs` (`REG_GOAL_CURRENT`, `0x138`).
    JointTorques
}

/// Cached present-state snapshot published on the telemetry Rx channel.
///
/// Each array is indexed by discovered joint slot. Unequipped channels (no
/// capability bit, or `0xFFFF` temperature sentinel) carry `NaN`.
///
/// Upstream: `barefoc-master/src/registers.rs` (`REG_PRESENT_POSITION`,
/// `REG_PRESENT_VELOCITY`, `REG_PRESENT_CURRENT`), `bareFOC/src/control_table.rs`
/// (`REG_PRESENT_TEMPERATURE`).
#[derive(Clone, Debug, Default, Reflect, Serialize, Deserialize)]
pub struct JointTelemetry {
    /// Measured joint angles in the bridge's configured angular unit.
    pub positions: CuArray<f32, MAX_ACTUATORS>,
    /// Measured joint velocities in the bridge's configured angular unit per second.
    pub velocities: CuArray<f32, MAX_ACTUATORS>,
    /// Measured motor currents in amperes (`NaN` when current sense is absent).
    pub currents: CuArray<f32, MAX_ACTUATORS>,
    /// Measured stage temperatures in degrees Celsius (`NaN` when unequipped).
    pub temperatures: CuArray<f32, MAX_ACTUATORS>,
}

impl JointTelemetry {
    /// Replace all four channels from fixed arrays, keeping only `count` joints.
    pub(crate) fn set_from_arrays(
        &mut self,
        count: usize,
        positions: [f32; MAX_ACTUATORS],
        velocities: [f32; MAX_ACTUATORS],
        currents: [f32; MAX_ACTUATORS],
        temperatures: [f32; MAX_ACTUATORS],
    ) {
        self.positions
            .fill_from_iter(positions.into_iter().take(count));
        self.velocities
            .fill_from_iter(velocities.into_iter().take(count));
        self.currents
            .fill_from_iter(currents.into_iter().take(count));
        self.temperatures
            .fill_from_iter(temperatures.into_iter().take(count));
    }
}

impl PartialEq for JointTelemetry {
    fn eq(&self, other: &Self) -> bool {
        self.positions.as_slice() == other.positions.as_slice()
            && self.velocities.as_slice() == other.velocities.as_slice()
            && self.currents.as_slice() == other.currents.as_slice()
            && self.temperatures.as_slice() == other.temperatures.as_slice()
    }
}

impl Encode for JointTelemetry {
    fn encode<E: Encoder>(&self, encoder: &mut E) -> Result<(), EncodeError> {
        Encode::encode(&self.positions, encoder)?;
        Encode::encode(&self.velocities, encoder)?;
        Encode::encode(&self.currents, encoder)?;
        Encode::encode(&self.temperatures, encoder)
    }
}

impl Decode<()> for JointTelemetry {
    fn decode<D: Decoder<Context = ()>>(decoder: &mut D) -> Result<Self, DecodeError> {
        Ok(Self {
            positions: Decode::decode(decoder)?,
            velocities: Decode::decode(decoder)?,
            currents: Decode::decode(decoder)?,
            temperatures: Decode::decode(decoder)?,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn round_trip<T>(value: &T) -> T
    where
        T: Encode + Decode<()>,
    {
        let bytes =
            cu29::bincode::encode_to_vec(value, cu29::bincode::config::standard()).expect("encode");
        let (decoded, used): (T, usize) =
            cu29::bincode::decode_from_slice(&bytes, cu29::bincode::config::standard())
                .expect("decode");
        assert_eq!(used, bytes.len());
        decoded
    }

    #[test]
    fn messages_round_trip_all_values() {
        let positions = JointPositions::from_slice(&[0.0, 1.5, -2.25]).unwrap();
        assert_eq!(round_trip(&positions), positions);
        let velocities = JointVelocities::from_slice(&[3.5, -0.5]).unwrap();
        assert_eq!(round_trip(&velocities), velocities);
        let torques = JointTorques::from_slice(&[0.125]).unwrap();
        assert_eq!(round_trip(&torques), torques);

        let mut telemetry = JointTelemetry::default();
        let mut positions = [0.0f32; MAX_ACTUATORS];
        positions[0] = 1.0;
        positions[1] = 2.0;
        let mut currents = [0.0f32; MAX_ACTUATORS];
        currents[0] = f32::NAN;
        currents[1] = 0.5;
        let mut temperatures = [0.0f32; MAX_ACTUATORS];
        temperatures[0] = 25.0;
        temperatures[1] = f32::NAN;
        telemetry.set_from_arrays(2, positions, [0.0; MAX_ACTUATORS], currents, temperatures);
        let decoded = round_trip(&telemetry);
        assert_eq!(decoded.positions.as_slice(), &[1.0, 2.0]);
        assert!(decoded.currents.as_slice()[0].is_nan());
        assert!(decoded.temperatures.as_slice()[1].is_nan());
    }

    #[test]
    fn capacity_is_enforced_without_truncation() {
        let too_many = [1.0f32; MAX_ACTUATORS + 1];
        assert_eq!(
            JointPositions::from_slice(&too_many),
            Err(BridgeError::Capacity {
                requested: MAX_ACTUATORS + 1,
                maximum: MAX_ACTUATORS,
            })
        );
        let at_capacity = [1.0f32; MAX_ACTUATORS];
        assert_eq!(
            JointTorques::from_slice(&at_capacity).unwrap().len(),
            MAX_ACTUATORS
        );
    }
}
