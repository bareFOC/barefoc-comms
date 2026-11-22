//! # Generic SimpleFOC register profile
//!
//! Auto-generates a [`RegMap`] descriptor set from any type implementing
//! [`barefoc_drivers::comms::RegisterBridge`], exposing the full SimpleFOC
//! runtime-configurable register surface (motor params, PID gains, limits,
//! driver params, current-sense gains/offsets, sensor params, and kinematics
//! extensions) over any address-based bus without hand-written descriptors.

use barefoc_drivers::comms::registers::{RegisterId, RegisterValue};
use barefoc_drivers::comms::RegisterBridge;

use crate::table::{HookResult, RegEntry, RegMap};

/// Universal register offset for the multi-motor selector (0x70).
pub const REG_MOTOR_SELECT: u16 = 0x70;

/// Universal register offset for SaveConfig command trigger (0x64).
pub const REG_SAVE_CONFIG: u16 = 0x64;

/// Universal register offset for TopologyMode selection (0x78).
pub const REG_TOPOLOGY_MODE: u16 = 0x78;

/// Generic write hook for a SimpleFOC register. Deserializes the bank bytes into a
/// [`RegisterValue`], calls [`RegisterBridge::write_reg`], and returns
/// [`HookResult::Applied`]. On any deserialization or bridge error the write is
/// silently skipped (the bank still holds the bytes so read-back returns what the
/// host sent).
fn sf_hook<B: RegisterBridge>(ctx: &mut B, offset: u16, bytes: &[u8]) -> HookResult {
    let id = match RegisterId::try_from(offset as u8) {
        Ok(id) => id,
        Err(_) => return HookResult::Applied,
    };
    let value = match RegisterValue::from_bytes(id, bytes) {
        Ok(v) => v,
        Err(_) => return HookResult::Applied,
    };
    let _ = ctx.write_reg(id, value);
    HookResult::Applied
}

/// Generic write hook for `REG_SAVE_CONFIG` (0x64).
pub fn save_config_hook<B: RegisterBridge>(
    _ctx: &mut B,
    _offset: u16,
    _bytes: &[u8],
) -> HookResult {
    HookResult::Applied
}

/// Auto-generate a [`RegMap`] from a [`RegisterBridge`] implementation.
pub fn simplefoc_profile<B: RegisterBridge>(
    bridge: B,
) -> RegMap<B, 384, 128> {
    let mut map = RegMap::new(bridge);
    for id in RegisterId::ALL {
        if id == RegisterId::Position {
            continue;
        }
        let offset = id as u16;
        let width = id.expected_size() as u8;
        if offset == REG_SAVE_CONFIG {
            let hook: crate::table::WriteHookFn<B> = save_config_hook::<B>;
            let _ = map.add_register(RegEntry::rw_hooked(offset, 1, hook));
        } else if id.is_read_only() {
            let _ = map.add_register(RegEntry::ro(offset, width));
        } else {
            let hook: crate::table::WriteHookFn<B> = sf_hook::<B>;
            let _ = map.add_register(RegEntry::rw_hooked(offset, width, hook));
        }
    }
    if map.get(REG_SAVE_CONFIG, &mut [0u8; 1]).is_err() {
        let hook: crate::table::WriteHookFn<B> = save_config_hook::<B>;
        let _ = map.add_register(RegEntry::rw_hooked(REG_SAVE_CONFIG, 1, hook));
    }
    map
}

/// Multi-motor context holding an array of `N` motor register bridges and a `selected` index.
pub struct MultiMotorState<const N: usize, B: RegisterBridge> {
    /// Array of motor bridges.
    pub motors: [B; N],
    /// Currently selected motor index.
    pub selected: u8,
}

impl<const N: usize, B: RegisterBridge> MultiMotorState<N, B> {
    /// Create a new multi-motor state with `selected` defaulting to 0.
    pub const fn new(motors: [B; N]) -> Self {
        Self {
            motors,
            selected: 0,
        }
    }

    /// Return the currently selected motor index.
    pub const fn selected(&self) -> u8 {
        self.selected
    }

    /// Attempt to select a motor by index.
    ///
    /// Returns `true` if `(index as usize) < N` and sets `selected`, or `false`
    /// if `(index as usize) >= N` leaving `selected` unchanged.
    pub fn set_selected(&mut self, index: u8) -> bool {
        if (index as usize) < N {
            self.selected = index;
            true
        } else {
            false
        }
    }

    /// Borrow the currently selected motor bridge.
    pub fn selected_motor(&self) -> &B {
        &self.motors[self.selected as usize]
    }

    /// Mutably borrow the currently selected motor bridge.
    pub fn selected_motor_mut(&mut self) -> &mut B {
        &mut self.motors[self.selected as usize]
    }
}

/// Write hook for motor window registers (`0x00..=0x6C`).
/// Dispatches `write_reg` to `ctx.motors[ctx.selected]`.
fn multi_motor_sf_hook<const N: usize, B: RegisterBridge>(
    ctx: &mut MultiMotorState<N, B>,
    offset: u16,
    bytes: &[u8],
) -> HookResult {
    let id = match RegisterId::try_from(offset as u8) {
        Ok(id) => id,
        Err(_) => return HookResult::Applied,
    };
    let value = match RegisterValue::from_bytes(id, bytes) {
        Ok(v) => v,
        Err(_) => return HookResult::Applied,
    };
    let sel = ctx.selected as usize;
    if sel < N {
        let _ = ctx.motors[sel].write_reg(id, value);
    }
    HookResult::Applied
}

/// Write hook for `MotorSelect` (`0x70`).
/// Updates `ctx.selected` if index < N; otherwise self-clears to 0.
fn motor_select_hook<const N: usize, B: RegisterBridge>(
    ctx: &mut MultiMotorState<N, B>,
    _offset: u16,
    bytes: &[u8],
) -> HookResult {
    if bytes.is_empty() {
        return HookResult::SelfClear;
    }
    if ctx.set_selected(bytes[0]) {
        HookResult::Applied
    } else {
        HookResult::SelfClear
    }
}

/// Write hook for board and kinematics registers (`0x71..=0xBF`).
/// Dispatches `write_reg` to global state (`ctx.motors[0]`).
fn multi_global_sf_hook<const N: usize, B: RegisterBridge>(
    ctx: &mut MultiMotorState<N, B>,
    offset: u16,
    bytes: &[u8],
) -> HookResult {
    let id = match RegisterId::try_from(offset as u8) {
        Ok(id) => id,
        Err(_) => return HookResult::Applied,
    };
    let value = match RegisterValue::from_bytes(id, bytes) {
        Ok(v) => v,
        Err(_) => return HookResult::Applied,
    };
    if N > 0 {
        let _ = ctx.motors[0].write_reg(id, value);
    }
    HookResult::Applied
}

/// Write hook for `REG_SAVE_CONFIG` (0x64) in multi-motor profiles.
pub fn multi_motor_save_config_hook<const N: usize, B: RegisterBridge>(
    _ctx: &mut MultiMotorState<N, B>,
    _offset: u16,
    _bytes: &[u8],
) -> HookResult {
    HookResult::Applied
}

/// Write hook for `REG_TOPOLOGY_MODE` (0x78) in multi-motor profiles.
pub fn topology_mode_hook<const N: usize, B: RegisterBridge>(
    _ctx: &mut MultiMotorState<N, B>,
    _offset: u16,
    _bytes: &[u8],
) -> HookResult {
    HookResult::Applied
}

/// Register a custom save configuration hook on register `0x64`.
pub fn register_save_config_hook<CTX, const CAP: usize, const MAX: usize>(
    map: &mut RegMap<CTX, CAP, MAX>,
    hook: crate::table::WriteHookFn<CTX>,
) -> Result<(), crate::table::Error> {
    map.add_register(RegEntry::rw_hooked(REG_SAVE_CONFIG, 1, hook))
}

/// Register a custom topology mode hook on register `0x78`.
pub fn register_topology_mode_hook<CTX, const CAP: usize, const MAX: usize>(
    map: &mut RegMap<CTX, CAP, MAX>,
    hook: crate::table::WriteHookFn<CTX>,
) -> Result<(), crate::table::Error> {
    map.add_register(RegEntry::rw_hooked(REG_TOPOLOGY_MODE, 1, hook))
}

/// Auto-generate a [`RegMap`] for an N-motor system from an array of [`RegisterBridge`] instances.
///
/// Registers the motor window (`0x00..=0x6C`) routing to the currently selected motor,
/// `MotorSelect` (`0x70`) gating selector updates with bounds checking,
/// `SaveConfig` (`0x64`) for non-volatile persistence triggers,
/// `TopologyMode` (`0x78`) for dynamic kinematic topology reconfiguration,
/// and board/kinematics descriptors (`0x71..=0xBF`) routing to global state.
pub fn multi_motor_profile<const N: usize, B: RegisterBridge>(
    motors: [B; N],
) -> RegMap<MultiMotorState<N, B>, 384, 128> {
    let mut map = RegMap::new(MultiMotorState::new(motors));
    for id in RegisterId::ALL {
        if id == RegisterId::Position {
            continue;
        }
        let offset = id as u16;
        let width = id.expected_size() as u8;
        if offset == REG_MOTOR_SELECT {
            let hook: crate::table::WriteHookFn<MultiMotorState<N, B>> = motor_select_hook::<N, B>;
            let _ = map.add_register(RegEntry::rw_hooked(offset, 1, hook));
        } else if offset == REG_SAVE_CONFIG {
            let hook: crate::table::WriteHookFn<MultiMotorState<N, B>> = multi_motor_save_config_hook::<N, B>;
            let _ = map.add_register(RegEntry::rw_hooked(offset, 1, hook));
        } else if offset == REG_TOPOLOGY_MODE {
            let hook: crate::table::WriteHookFn<MultiMotorState<N, B>> = topology_mode_hook::<N, B>;
            let _ = map.add_register(RegEntry::rw_hooked(offset, 1, hook));
        } else if offset <= 0x6C {
            if id.is_read_only() {
                let _ = map.add_register(RegEntry::ro(offset, width));
            } else {
                let hook: crate::table::WriteHookFn<MultiMotorState<N, B>> = multi_motor_sf_hook::<N, B>;
                let _ = map.add_register(RegEntry::rw_hooked(offset, width, hook));
            }
        } else {
            if id.is_read_only() {
                let _ = map.add_register(RegEntry::ro(offset, width));
            } else {
                let hook: crate::table::WriteHookFn<MultiMotorState<N, B>> = multi_global_sf_hook::<N, B>;
                let _ = map.add_register(RegEntry::rw_hooked(offset, width, hook));
            }
        }
    }
    // Ensure REG_MOTOR_SELECT (0x70) is registered
    if map.get(REG_MOTOR_SELECT, &mut [0u8; 1]).is_err() {
        let hook: crate::table::WriteHookFn<MultiMotorState<N, B>> = motor_select_hook::<N, B>;
        let _ = map.add_register(RegEntry::rw_hooked(REG_MOTOR_SELECT, 1, hook));
    }
    // Ensure REG_SAVE_CONFIG (0x64) is registered
    if map.get(REG_SAVE_CONFIG, &mut [0u8; 1]).is_err() {
        let hook: crate::table::WriteHookFn<MultiMotorState<N, B>> = multi_motor_save_config_hook::<N, B>;
        let _ = map.add_register(RegEntry::rw_hooked(REG_SAVE_CONFIG, 1, hook));
    }
    // Ensure REG_TOPOLOGY_MODE (0x78) is registered
    if map.get(REG_TOPOLOGY_MODE, &mut [0u8; 1]).is_err() {
        let hook: crate::table::WriteHookFn<MultiMotorState<N, B>> = topology_mode_hook::<N, B>;
        let _ = map.add_register(RegEntry::rw_hooked(REG_TOPOLOGY_MODE, 1, hook));
    }
    map
}

/// Refresh all read-only register bytes in the bank from the bridge.
pub fn refresh_telemetry<B: RegisterBridge, const CAP: usize, const MAX: usize>(
    map: &mut RegMap<B, CAP, MAX>,
) {
    for id in RegisterId::ALL {
        if id.is_read_only() {
            let offset = id as u16;
            let width = id.expected_size();
            if let Ok(val) = map.handler().read_reg(id) {
                let mut buf = [0u8; 12];
                if let Ok(n) = val.to_bytes(&mut buf) {
                    let _ = map.set(offset, &buf[..width.min(n)]);
                }
            }
        }
    }
}

/// Refresh all read-only register bytes in the bank from the currently selected motor.
///
/// Motor-window read-only registers are queried from `motors[selected]`.
/// Board and kinematics read-only registers (0x71..=0xBF) are queried from global state (`motors[0]`).
pub fn refresh_telemetry_multi<const N: usize, B: RegisterBridge, const CAP: usize, const MAX: usize>(
    map: &mut RegMap<MultiMotorState<N, B>, CAP, MAX>,
) {
    let selected = map.handler().selected as usize;
    if selected >= N {
        return;
    }
    for id in RegisterId::ALL {
        let offset = id as u16;
        if id.is_read_only() && offset != REG_MOTOR_SELECT {
            let width = id.expected_size();
            let res = if offset <= 0x6C {
                map.handler().motors[selected].read_reg(id)
            } else {
                map.handler().motors[0].read_reg(id)
            };
            if let Ok(val) = res {
                let mut buf = [0u8; 12];
                if let Ok(n) = val.to_bytes(&mut buf) {
                    let _ = map.set(offset, &buf[..width.min(n)]);
                }
            }
        }
    }
}

#[cfg(all(test, feature = "std"))]
mod tests {
    use super::*;
    use barefoc::motor::base::MotorConfig;
    use barefoc::motor::base::NullClock;
    use crate::client::RegClientI2c;
    use crate::table::Error as MapError;
    use crate::transport::{RegServer, TargetTransport};
    use embedded_hal::i2c as eh_i2c;
    use embedded_hal::i2c::I2c;
    use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTx};

    const DEV: u8 = 0x30;

    struct FakeI2cBus<'a, C, const CAP: usize, const MAX: usize> {
        server: &'a mut RegServer<C, CAP, MAX>,
    }

    impl<'a, C, const CAP: usize, const MAX: usize> eh_i2c::ErrorType for FakeI2cBus<'a, C, CAP, MAX> {
        type Error = eh_i2c::ErrorKind;
    }

    impl<'a, C, const CAP: usize, const MAX: usize> I2c for FakeI2cBus<'a, C, CAP, MAX> {
        fn write(&mut self, _address: u8, bytes: &[u8]) -> Result<(), Self::Error> {
            self.server
                .on_write(bytes)
                .map_err(|_| eh_i2c::ErrorKind::NoAcknowledge(eh_i2c::NoAcknowledgeSource::Address))
        }

        fn read(&mut self, _address: u8, buf: &mut [u8]) -> Result<(), Self::Error> {
            match self.server.on_read(buf.len(), buf) {
                Ok(_) => Ok(()),
                Err(MapError::BufferTooSmall) => Err(eh_i2c::ErrorKind::Overrun),
                Err(_) => Err(eh_i2c::ErrorKind::NoAcknowledge(eh_i2c::NoAcknowledgeSource::Address)),
            }
        }

        fn write_read(
            &mut self,
            address: u8,
            initial: &[u8],
            buffer: &mut [u8],
        ) -> Result<(), Self::Error> {
            self.write(address, initial)?;
            self.read(address, buffer)
        }

        fn transaction(
            &mut self,
            address: u8,
            operations: &mut [eh_i2c::Operation<'_>],
        ) -> Result<(), Self::Error> {
            for op in operations.iter_mut() {
                match op {
                    eh_i2c::Operation::Write(b) => self.write(address, b)?,
                    eh_i2c::Operation::Read(b) => self.read(address, b)?,
                }
            }
            Ok(())
        }
    }

    #[test]
    fn simplefoc_profile_entry_count() {
        let clock = NullClock;
        let config = MotorConfig::new(&clock);
        let map = simplefoc_profile(config);
        assert!(map.extent() >= 0x6C);
    }

    #[test]
    fn simplefoc_profile_ro_rejection() {
        let clock = NullClock;
        let config = MotorConfig::new(&clock);
        let mut map = simplefoc_profile(config);
        // Pointer 0x0000 with a one-byte payload at a read-only register.
        let res = map.write_phase(&[0x00, 0x00, 0x01]);
        assert!(res.is_err());
    }

    #[test]
    fn simplefoc_profile_rw_roundtrip() {
        let clock = NullClock;
        let config = MotorConfig::new(&clock);
        let mut map = simplefoc_profile(config);

        let val: f32 = 12.5;
        let bytes = val.to_le_bytes();
        let mut phase = [0u8; 6];
        phase[0] = 0x00; phase[1] = 0x50; // VoltageLimit
        phase[2..].copy_from_slice(&bytes);
        map.write_phase(&phase).unwrap();

        map.write_phase(&[0x00, 0x50]).unwrap();
        let mut out = [0u8; 4];
        let n = map.read_phase(4, &mut out).unwrap();
        assert_eq!(n, 4);
        assert_eq!(f32::from_le_bytes(out), 12.5);
    }

    #[test]
    fn multi_motor_state_bounds_and_selector() {
        let clock = NullClock;
        let config0 = MotorConfig::new(&clock);
        let config1 = MotorConfig::new(&clock);
        let mut state = MultiMotorState::<2, _>::new([config0, config1]);

        assert_eq!(state.selected(), 0);
        assert!(state.set_selected(1));
        assert_eq!(state.selected(), 1);
        assert!(!state.set_selected(2));
        assert_eq!(state.selected(), 1);
        assert!(!state.set_selected(99));
        assert_eq!(state.selected(), 1);
        assert!(state.set_selected(0));
        assert_eq!(state.selected(), 0);
    }

    #[test]
    fn multi_motor_target_and_gain_separation() {
        let clock = NullClock;
        let config0 = MotorConfig::new(&clock);
        let config1 = MotorConfig::new(&clock);
        let mut map = multi_motor_profile([config0, config1]);

        // Default selected is 0: write target and gain to motor 0
        let target0 = 5.0f32;
        let mut phase = [0u8; 6];
        phase[0] = 0x00; phase[1] = 0x01; // Target
        phase[2..].copy_from_slice(&target0.to_le_bytes());
        map.write_phase(&phase).unwrap();

        let gain0 = 1.25f32;
        let mut phase_gain = [0u8; 6];
        phase_gain[0] = 0x00; phase_gain[1] = 0x30; // VelPidP
        phase_gain[2..].copy_from_slice(&gain0.to_le_bytes());
        map.write_phase(&phase_gain).unwrap();

        // Select motor 1: write 1 to MotorSelect (0x70)
        map.write_phase(&[0x00, 0x70, 0x01]).unwrap();
        assert_eq!(map.handler().selected(), 1);

        // Write target and gain to motor 1
        let target1 = 10.0f32;
        phase[0] = 0x00; phase[1] = 0x01;
        phase[2..].copy_from_slice(&target1.to_le_bytes());
        map.write_phase(&phase).unwrap();

        let gain1 = 3.5f32;
        phase_gain[0] = 0x00; phase_gain[1] = 0x30;
        phase_gain[2..].copy_from_slice(&gain1.to_le_bytes());
        map.write_phase(&phase_gain).unwrap();

        // Verify motor 0 has target0 and gain0; motor 1 has target1 and gain1
        assert_eq!(map.handler().motors[0].target, 5.0);
        assert_eq!(map.handler().motors[0].PID_velocity.p(), 1.25);
        assert_eq!(map.handler().motors[1].target, 10.0);
        assert_eq!(map.handler().motors[1].PID_velocity.p(), 3.5);
    }

    #[test]
    fn multi_motor_selector_bounds_rejection() {
        let clock = NullClock;
        let config0 = MotorConfig::new(&clock);
        let config1 = MotorConfig::new(&clock);
        let mut map = multi_motor_profile([config0, config1]);

        // Select motor 1 (valid)
        map.write_phase(&[0x00, 0x70, 0x01]).unwrap();
        assert_eq!(map.handler().selected(), 1);

        // Write out-of-range selector (2 for N=2)
        map.write_phase(&[0x00, 0x70, 0x02]).unwrap();
        // State remains 1 (no state change to selected)
        assert_eq!(map.handler().selected(), 1);

        // Bank at 0x70 self-cleared
        let mut sel_byte = [0u8; 1];
        let _ = map.get(0x70, &mut sel_byte);
        assert_eq!(sel_byte[0], 0);
    }

    #[test]
    fn multi_motor_telemetry_refresh() {
        let clock = NullClock;
        let mut config0 = MotorConfig::new(&clock);
        let mut config1 = MotorConfig::new(&clock);
        config0.shaft_angle = 1.5;
        config0.shaft_velocity = 2.5;
        config1.shaft_angle = 3.5;
        config1.shaft_velocity = 4.5;

        let mut map = multi_motor_profile([config0, config1]);

        // Default selected is 0
        refresh_telemetry_multi(&mut map);

        map.write_phase(&[0x00, 0x09]).unwrap();
        let mut angle_bytes = [0u8; 4];
        map.read_phase(4, &mut angle_bytes).unwrap();
        assert_eq!(f32::from_le_bytes(angle_bytes), 1.5);

        map.write_phase(&[0x00, 0x11]).unwrap();
        let mut vel_bytes = [0u8; 4];
        map.read_phase(4, &mut vel_bytes).unwrap();
        assert_eq!(f32::from_le_bytes(vel_bytes), 2.5);

        // Switch to motor 1
        map.write_phase(&[0x00, 0x70, 0x01]).unwrap();
        refresh_telemetry_multi(&mut map);

        map.write_phase(&[0x00, 0x09]).unwrap();
        map.read_phase(4, &mut angle_bytes).unwrap();
        assert_eq!(f32::from_le_bytes(angle_bytes), 3.5);

        map.write_phase(&[0x00, 0x11]).unwrap();
        map.read_phase(4, &mut vel_bytes).unwrap();
        assert_eq!(f32::from_le_bytes(vel_bytes), 4.5);
    }

    #[test]
    fn multi_motor_mock_i2c_client() {
        let clock = NullClock;
        let mut config0 = MotorConfig::new(&clock);
        let mut config1 = MotorConfig::new(&clock);
        config0.shaft_angle = 1.23;
        config0.shaft_velocity = 4.56;
        config1.shaft_angle = 7.89;
        config1.shaft_velocity = 0.12;

        let map = multi_motor_profile([config0, config1]);
        let mut server = RegServer::new(map);

        {
            let mut bus = FakeI2cBus {
                server: &mut server,
            };
            let mut client = RegClientI2c::<_, 8>::new(&mut bus);

            // 1. Read motor 0 telemetry over I2C
            client.write_register(DEV, 0x70, &[0]).unwrap();
        }
        refresh_telemetry_multi(server.map_mut());
        {
            let mut bus = FakeI2cBus {
                server: &mut server,
            };
            let mut client = RegClientI2c::<_, 8>::new(&mut bus);
            let mut angle_buf = [0u8; 4];
            let mut vel_buf = [0u8; 4];
            client
                .read_register(DEV, 0x09, 4, &mut angle_buf)
                .unwrap();
            client
                .read_register(DEV, 0x11, 4, &mut vel_buf)
                .unwrap();
            assert_eq!(f32::from_le_bytes(angle_buf), 1.23);
            assert_eq!(f32::from_le_bytes(vel_buf), 4.56);

            // 2. Select motor 1 over I2C
            client.write_register(DEV, 0x70, &[1]).unwrap();
        }
        refresh_telemetry_multi(server.map_mut());
        {
            let mut bus = FakeI2cBus {
                server: &mut server,
            };
            let mut client = RegClientI2c::<_, 8>::new(&mut bus);
            let mut angle_buf = [0u8; 4];
            let mut vel_buf = [0u8; 4];
            client
                .read_register(DEV, 0x09, 4, &mut angle_buf)
                .unwrap();
            client
                .read_register(DEV, 0x11, 4, &mut vel_buf)
                .unwrap();
            assert_eq!(f32::from_le_bytes(angle_buf), 7.89);
            assert_eq!(f32::from_le_bytes(vel_buf), 0.12);
        }
    }

    #[test]
    fn multi_motor_scripted_mock_i2c_wire_transactions() {
        let angle0 = 1.23f32.to_le_bytes();
        let angle1 = 7.89f32.to_le_bytes();

        let expectations = [
            // Select motor 0
            I2cTx::write(DEV, vec![0x00, 0x70, 0]),
            // Pointer latch Angle (0x09)
            I2cTx::write(DEV, vec![0x00, 0x09]),
            // Read 4 bytes angle0
            I2cTx::read(DEV, angle0.to_vec()),
            // Select motor 1
            I2cTx::write(DEV, vec![0x00, 0x70, 1]),
            // Pointer latch Angle (0x09)
            I2cTx::write(DEV, vec![0x00, 0x09]),
            // Read 4 bytes angle1
            I2cTx::read(DEV, angle1.to_vec()),
        ];

        let mut mock_bus = I2cMock::new(&expectations);
        {
            let mut client = RegClientI2c::<_, 8>::new(&mut mock_bus);
            let mut buf = [0u8; 4];

            client.write_register(DEV, 0x70, &[0]).unwrap();
            client.read_register(DEV, 0x09, 4, &mut buf).unwrap();
            assert_eq!(f32::from_le_bytes(buf), 1.23);

            client.write_register(DEV, 0x70, &[1]).unwrap();
            client.read_register(DEV, 0x09, 4, &mut buf).unwrap();
            assert_eq!(f32::from_le_bytes(buf), 7.89);
        }
        mock_bus.done();
    }

    #[test]
    fn test_save_config_and_topology_mode_registers() {
        let clock = NullClock;
        let motor0 = MotorConfig::new(&clock);
        let motor1 = MotorConfig::new(&clock);
        let mut server = RegServer::new(multi_motor_profile([motor0, motor1]));

        let mut bus = FakeI2cBus {
            server: &mut server,
        };
        let mut client = RegClientI2c::<_, 8>::new(&mut bus);

        // Write 0x55 (Save Config command) to 0x64
        client.write_register(DEV, REG_SAVE_CONFIG, &[0x55]).unwrap();
        let mut save_val = [0u8; 1];
        client.read_register(DEV, REG_SAVE_CONFIG, 1, &mut save_val).unwrap();
        assert_eq!(save_val[0], 0x55);

        // Write 0x02 (Topology Mode) to 0x78
        client.write_register(DEV, REG_TOPOLOGY_MODE, &[0x02]).unwrap();
        let mut topo_val = [0u8; 1];
        client.read_register(DEV, REG_TOPOLOGY_MODE, 1, &mut topo_val).unwrap();
        assert_eq!(topo_val[0], 0x02);
    }
}
