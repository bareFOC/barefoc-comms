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

/// Universal register offset for the multi-motor selector (`MotorSelect`, 4-byte aligned).
pub const REG_MOTOR_SELECT: u16 = RegisterId::MotorSelect.aligned_offset();

/// Universal register offset for `CalibCommand` (4-byte aligned).
pub const REG_CALIB_CMD: u16 = RegisterId::CalibCommand.aligned_offset();

/// Universal register offset for SaveConfig command trigger (non-overlapping, 4-byte aligned).
pub const REG_SAVE_CONFIG: u16 = RegisterId::ALIGNED_SPAN;

/// Universal register offset for TopologyMode selection (non-overlapping, 4-byte aligned).
pub const REG_TOPOLOGY_MODE: u16 = RegisterId::ALIGNED_SPAN + 4;

/// Universal register offset for 4-channel SensorMap `[u8; 4]` (non-overlapping, 4-byte aligned).
pub const REG_SENSOR_MAP: u16 = RegisterId::ALIGNED_SPAN + 8;

/// Return the 4-byte-aligned, non-overlapping byte offset for `id`.
#[inline]
pub const fn reg_offset(id: RegisterId) -> u16 {
    id.aligned_offset()
}

/// Generic write hook for a SimpleFOC register. Deserializes the bank bytes into a
/// [`RegisterValue`], calls [`RegisterBridge::write_reg`], and returns
/// [`HookResult::Applied`]. On any deserialization or bridge error the write is
/// silently skipped (the bank still holds the bytes so read-back returns what the
/// host sent).
fn sf_hook<B: RegisterBridge>(ctx: &mut B, offset: u16, bytes: &[u8]) -> HookResult {
    let id = match RegisterId::from_aligned_offset(offset) {
        Some(id) => id,
        None => return HookResult::Applied,
    };
    let value = match RegisterValue::from_bytes(id, bytes) {
        Ok(v) => v,
        Err(_) => return HookResult::Applied,
    };
    let _ = ctx.write_reg(id, value);
    HookResult::Applied
}

/// Generic write hook for `REG_SAVE_CONFIG`.
pub fn save_config_hook<B: RegisterBridge>(
    _ctx: &mut B,
    _offset: u16,
    _bytes: &[u8],
) -> HookResult {
    HookResult::Applied
}

/// Auto-generate a [`RegMap`] from a [`RegisterBridge`] implementation.
///
/// All registers are placed on strict 4-byte-aligned, non-overlapping boundaries.
/// Panics if any register fails registration.
pub fn simplefoc_profile<B: RegisterBridge>(
    bridge: B,
) -> RegMap<B, 640, 160> {
    let mut map = RegMap::new(bridge);
    for id in RegisterId::ALL {
        if id == RegisterId::Position {
            continue;
        }
        let offset = id.aligned_offset();
        let width = id.expected_size() as u8;
        if id.is_read_only() {
            map.add_register(RegEntry::ro(offset, width))
                .expect("simplefoc_profile RO register overlap or capacity exceeded");
        } else {
            let hook: crate::table::WriteHookFn<B> = sf_hook::<B>;
            map.add_register(RegEntry::rw_hooked(offset, width, hook))
                .expect("simplefoc_profile RW register overlap or capacity exceeded");
        }
    }
    let hook: crate::table::WriteHookFn<B> = save_config_hook::<B>;
    map.add_register(RegEntry::rw_hooked(REG_SAVE_CONFIG, 1, hook))
        .expect("simplefoc_profile REG_SAVE_CONFIG registration failed");
    sync_rw_from_bridge(&mut map);
    map
}

/// Multi-motor context holding an array of `N` motor register bridges and a `selected` index.
pub struct MultiMotorState<const N: usize, B: RegisterBridge> {
    /// Array of motor bridges.
    pub motors: [B; N],
    /// Currently selected motor index.
    pub selected: u8,
    /// Set when `selected` changes so the next `refresh_telemetry_multi` re-seeds R/W slots.
    pub rw_dirty: bool,
}

impl<const N: usize, B: RegisterBridge> MultiMotorState<N, B> {
    /// Create a new multi-motor state with `selected` defaulting to 0.
    pub const fn new(motors: [B; N]) -> Self {
        Self {
            motors,
            selected: 0,
            rw_dirty: false,
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
            if self.selected != index {
                self.selected = index;
                self.rw_dirty = true;
            }
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

/// Write hook for motor window registers (`RegisterId <= 0x6C`).
/// Dispatches `write_reg` to `ctx.motors[ctx.selected]`.
fn multi_motor_sf_hook<const N: usize, B: RegisterBridge>(
    ctx: &mut MultiMotorState<N, B>,
    offset: u16,
    bytes: &[u8],
) -> HookResult {
    let id = match RegisterId::from_aligned_offset(offset) {
        Some(id) => id,
        None => return HookResult::Applied,
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

/// Write hook for `MotorSelect` (`REG_MOTOR_SELECT`).
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

/// Write hook for board and kinematics registers (`RegisterId > 0x6C`).
/// Dispatches `write_reg` to global state (`ctx.motors[0]`).
fn multi_global_sf_hook<const N: usize, B: RegisterBridge>(
    ctx: &mut MultiMotorState<N, B>,
    offset: u16,
    bytes: &[u8],
) -> HookResult {
    let id = match RegisterId::from_aligned_offset(offset) {
        Some(id) => id,
        None => return HookResult::Applied,
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

/// Write hook for `REG_SENSOR_MAP` in multi-motor profiles.
pub fn sensor_map_hook<const N: usize, B: RegisterBridge>(
    _ctx: &mut MultiMotorState<N, B>,
    _offset: u16,
    _bytes: &[u8],
) -> HookResult {
    HookResult::Applied
}

/// Register a custom save configuration hook on `REG_SAVE_CONFIG`.
pub fn register_save_config_hook<CTX, const CAP: usize, const MAX: usize>(
    map: &mut RegMap<CTX, CAP, MAX>,
    hook: crate::table::WriteHookFn<CTX>,
) -> Result<(), crate::table::Error> {
    map.set_write_hook(REG_SAVE_CONFIG, hook)
}

/// Register a custom topology mode hook on `REG_TOPOLOGY_MODE`.
pub fn register_topology_mode_hook<CTX, const CAP: usize, const MAX: usize>(
    map: &mut RegMap<CTX, CAP, MAX>,
    hook: crate::table::WriteHookFn<CTX>,
) -> Result<(), crate::table::Error> {
    map.set_write_hook(REG_TOPOLOGY_MODE, hook)
}

/// Register a custom sensor map hook on `REG_SENSOR_MAP`.
pub fn register_sensor_map_hook<CTX, const CAP: usize, const MAX: usize>(
    map: &mut RegMap<CTX, CAP, MAX>,
    hook: crate::table::WriteHookFn<CTX>,
) -> Result<(), crate::table::Error> {
    map.set_write_hook(REG_SENSOR_MAP, hook)
}

/// Auto-generate a [`RegMap`] for an N-motor system from an array of [`RegisterBridge`] instances.
///
/// Registers are placed on strict 4-byte-aligned, non-overlapping boundaries.
/// Panics if any register fails registration (zero silent drops).
pub fn multi_motor_profile<const N: usize, B: RegisterBridge>(
    motors: [B; N],
) -> RegMap<MultiMotorState<N, B>, 640, 160> {
    let mut map = RegMap::new(MultiMotorState::new(motors));
    for id in RegisterId::ALL {
        if id == RegisterId::Position {
            continue;
        }
        let offset = id.aligned_offset();
        let width = id.expected_size() as u8;
        if offset == REG_MOTOR_SELECT {
            let hook: crate::table::WriteHookFn<MultiMotorState<N, B>> = motor_select_hook::<N, B>;
            map.add_register(RegEntry::rw_hooked(offset, 1, hook))
                .expect("multi_motor_profile REG_MOTOR_SELECT registration failed");
        } else if (id as u8) <= 0x6C {
            if id.is_read_only() {
                map.add_register(RegEntry::ro(offset, width))
                    .expect("multi_motor_profile motor RO register overlap or capacity exceeded");
            } else {
                let hook: crate::table::WriteHookFn<MultiMotorState<N, B>> = multi_motor_sf_hook::<N, B>;
                map.add_register(RegEntry::rw_hooked(offset, width, hook))
                    .expect("multi_motor_profile motor RW register overlap or capacity exceeded");
            }
        } else if id.is_read_only() {
            map.add_register(RegEntry::ro(offset, width))
                .expect("multi_motor_profile global RO register overlap or capacity exceeded");
        } else {
            let hook: crate::table::WriteHookFn<MultiMotorState<N, B>> = multi_global_sf_hook::<N, B>;
            map.add_register(RegEntry::rw_hooked(offset, width, hook))
                .expect("multi_motor_profile global RW register overlap or capacity exceeded");
        }
    }
    let save_hook: crate::table::WriteHookFn<MultiMotorState<N, B>> = multi_motor_save_config_hook::<N, B>;
    map.add_register(RegEntry::rw_hooked(REG_SAVE_CONFIG, 1, save_hook))
        .expect("multi_motor_profile REG_SAVE_CONFIG registration failed");
    let topo_hook: crate::table::WriteHookFn<MultiMotorState<N, B>> = topology_mode_hook::<N, B>;
    map.add_register(RegEntry::rw_hooked(REG_TOPOLOGY_MODE, 1, topo_hook))
        .expect("multi_motor_profile REG_TOPOLOGY_MODE registration failed");
    let smap_hook: crate::table::WriteHookFn<MultiMotorState<N, B>> = sensor_map_hook::<N, B>;
    map.add_register(RegEntry::rw_hooked(REG_SENSOR_MAP, 4, smap_hook))
        .expect("multi_motor_profile REG_SENSOR_MAP registration failed");
    sync_rw_from_bridge_multi(&mut map);
    map
}

/// Seed all R/W register bytes in `map` from [`RegisterBridge::read_reg`].
pub fn sync_rw_from_bridge<B: RegisterBridge, const CAP: usize, const MAX: usize>(
    map: &mut RegMap<B, CAP, MAX>,
) {
    for id in RegisterId::ALL {
        if id == RegisterId::Position
            || id.is_read_only()
            || id == RegisterId::MotorSelect
            || id == RegisterId::CalibCommand
        {
            continue;
        }
        let offset = id.aligned_offset();
        let width = id.expected_size();
        if let Ok(val) = map.handler().read_reg(id) {
            let mut buf = [0u8; 12];
            if let Ok(n) = val.to_bytes(&mut buf) {
                let _ = map.set(offset, &buf[..width.min(n)]);
            }
        }
    }
}

/// Seed all R/W register bytes in `map` from the currently selected motor's [`RegisterBridge::read_reg`].
pub fn sync_rw_from_bridge_multi<
    const N: usize,
    B: RegisterBridge,
    const CAP: usize,
    const MAX: usize,
>(
    map: &mut RegMap<MultiMotorState<N, B>, CAP, MAX>,
) {
    let selected = map.handler().selected as usize;
    if selected >= N {
        return;
    }
    for id in RegisterId::ALL {
        if id == RegisterId::Position
            || id.is_read_only()
            || id == RegisterId::MotorSelect
            || id == RegisterId::CalibCommand
        {
            continue;
        }
        let offset = id.aligned_offset();
        let width = id.expected_size();
        let res = if (id as u8) <= 0x6C {
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
    map.handler().rw_dirty = false;
}

/// Refresh all read-only register bytes in the bank from the bridge.
pub fn refresh_telemetry<B: RegisterBridge, const CAP: usize, const MAX: usize>(
    map: &mut RegMap<B, CAP, MAX>,
) {
    for id in RegisterId::ALL {
        if id.is_read_only() {
            let offset = id.aligned_offset();
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
/// Board and kinematics read-only registers (`RegisterId > 0x6C`) are queried from global state (`motors[0]`).
pub fn refresh_telemetry_multi<const N: usize, B: RegisterBridge, const CAP: usize, const MAX: usize>(
    map: &mut RegMap<MultiMotorState<N, B>, CAP, MAX>,
) {
    if map.handler().rw_dirty {
        sync_rw_from_bridge_multi(map);
    }
    let selected = map.handler().selected as usize;
    if selected >= N {
        return;
    }
    for id in RegisterId::ALL {
        let offset = id.aligned_offset();
        if id.is_read_only() && offset != REG_MOTOR_SELECT {
            let width = id.expected_size();
            let res = if (id as u8) <= 0x6C {
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
        let off = reg_offset(RegisterId::VoltageLimit).to_be_bytes();
        let mut phase = [0u8; 6];
        phase[0] = off[0]; phase[1] = off[1];
        phase[2..].copy_from_slice(&bytes);
        map.write_phase(&phase).unwrap();

        map.write_phase(&off).unwrap();
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
        let target_off = reg_offset(RegisterId::Target).to_be_bytes();
        let mut phase = [0u8; 6];
        phase[0] = target_off[0]; phase[1] = target_off[1];
        phase[2..].copy_from_slice(&target0.to_le_bytes());
        map.write_phase(&phase).unwrap();

        let gain0 = 1.25f32;
        let vel_p_off = reg_offset(RegisterId::VelPidP).to_be_bytes();
        let mut phase_gain = [0u8; 6];
        phase_gain[0] = vel_p_off[0]; phase_gain[1] = vel_p_off[1];
        phase_gain[2..].copy_from_slice(&gain0.to_le_bytes());
        map.write_phase(&phase_gain).unwrap();

        // Select motor 1: write 1 to MotorSelect
        let sel_off = REG_MOTOR_SELECT.to_be_bytes();
        map.write_phase(&[sel_off[0], sel_off[1], 0x01]).unwrap();
        assert_eq!(map.handler().selected(), 1);

        // Write target and gain to motor 1
        let target1 = 10.0f32;
        phase[0] = target_off[0]; phase[1] = target_off[1];
        phase[2..].copy_from_slice(&target1.to_le_bytes());
        map.write_phase(&phase).unwrap();

        let gain1 = 3.5f32;
        phase_gain[0] = vel_p_off[0]; phase_gain[1] = vel_p_off[1];
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

        let sel_off = REG_MOTOR_SELECT.to_be_bytes();
        // Select motor 1 (valid)
        map.write_phase(&[sel_off[0], sel_off[1], 0x01]).unwrap();
        assert_eq!(map.handler().selected(), 1);

        // Write out-of-range selector (2 for N=2)
        map.write_phase(&[sel_off[0], sel_off[1], 0x02]).unwrap();
        // State remains 1 (no state change to selected)
        assert_eq!(map.handler().selected(), 1);

        // Bank at REG_MOTOR_SELECT self-cleared
        let mut sel_byte = [0u8; 1];
        let _ = map.get(REG_MOTOR_SELECT, &mut sel_byte);
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
        let angle_off = reg_offset(RegisterId::Angle).to_be_bytes();
        let vel_off = reg_offset(RegisterId::Velocity).to_be_bytes();
        let sel_off = REG_MOTOR_SELECT.to_be_bytes();

        // Default selected is 0
        refresh_telemetry_multi(&mut map);

        map.write_phase(&angle_off).unwrap();
        let mut angle_bytes = [0u8; 4];
        map.read_phase(4, &mut angle_bytes).unwrap();
        assert_eq!(f32::from_le_bytes(angle_bytes), 1.5);

        map.write_phase(&vel_off).unwrap();
        let mut vel_bytes = [0u8; 4];
        map.read_phase(4, &mut vel_bytes).unwrap();
        assert_eq!(f32::from_le_bytes(vel_bytes), 2.5);

        // Switch to motor 1
        map.write_phase(&[sel_off[0], sel_off[1], 0x01]).unwrap();
        refresh_telemetry_multi(&mut map);

        map.write_phase(&angle_off).unwrap();
        map.read_phase(4, &mut angle_bytes).unwrap();
        assert_eq!(f32::from_le_bytes(angle_bytes), 3.5);

        map.write_phase(&vel_off).unwrap();
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
        let angle_off = reg_offset(RegisterId::Angle);
        let vel_off = reg_offset(RegisterId::Velocity);

        {
            let mut bus = FakeI2cBus {
                server: &mut server,
            };
            let mut client = RegClientI2c::<_, 8>::new(&mut bus);

            // 1. Read motor 0 telemetry over I2C
            client.write_register(DEV, REG_MOTOR_SELECT, &[0]).unwrap();
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
                .read_register(DEV, angle_off, 4, &mut angle_buf)
                .unwrap();
            client
                .read_register(DEV, vel_off, 4, &mut vel_buf)
                .unwrap();
            assert_eq!(f32::from_le_bytes(angle_buf), 1.23);
            assert_eq!(f32::from_le_bytes(vel_buf), 4.56);

            // 2. Select motor 1 over I2C
            client.write_register(DEV, REG_MOTOR_SELECT, &[1]).unwrap();
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
                .read_register(DEV, angle_off, 4, &mut angle_buf)
                .unwrap();
            client
                .read_register(DEV, vel_off, 4, &mut vel_buf)
                .unwrap();
            assert_eq!(f32::from_le_bytes(angle_buf), 7.89);
            assert_eq!(f32::from_le_bytes(vel_buf), 0.12);
        }
    }

    #[test]
    fn multi_motor_scripted_mock_i2c_wire_transactions() {
        let angle0 = 1.23f32.to_le_bytes();
        let angle1 = 7.89f32.to_le_bytes();
        let sel_be = REG_MOTOR_SELECT.to_be_bytes();
        let angle_be = reg_offset(RegisterId::Angle).to_be_bytes();

        let expectations = [
            // Select motor 0
            I2cTx::write(DEV, vec![sel_be[0], sel_be[1], 0]),
            // Combined write_read Angle (4 bytes angle0)
            I2cTx::write_read(DEV, vec![angle_be[0], angle_be[1]], angle0.to_vec()),
            // Select motor 1
            I2cTx::write(DEV, vec![sel_be[0], sel_be[1], 1]),
            // Combined write_read Angle (4 bytes angle1)
            I2cTx::write_read(DEV, vec![angle_be[0], angle_be[1]], angle1.to_vec()),
        ];

        let mut mock_bus = I2cMock::new(&expectations);
        {
            let mut client = RegClientI2c::<_, 8>::new(&mut mock_bus);
            let mut buf = [0u8; 4];

            client.write_register(DEV, REG_MOTOR_SELECT, &[0]).unwrap();
            client.read_register(DEV, reg_offset(RegisterId::Angle), 4, &mut buf).unwrap();
            assert_eq!(f32::from_le_bytes(buf), 1.23);

            client.write_register(DEV, REG_MOTOR_SELECT, &[1]).unwrap();
            client.read_register(DEV, reg_offset(RegisterId::Angle), 4, &mut buf).unwrap();
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

        // Write 0x02 (Topology Mode) to REG_TOPOLOGY_MODE
        client.write_register(DEV, REG_TOPOLOGY_MODE, &[0x02]).unwrap();
        let mut topo_val = [0u8; 1];
        client.read_register(DEV, REG_TOPOLOGY_MODE, 1, &mut topo_val).unwrap();
        assert_eq!(topo_val[0], 0x02);
    }

    #[test]
    fn test_aligned_layout_and_safety_tuning_roundtrip() {
        // Verify every RegisterId has a 4-byte aligned offset and zero overlaps.
        for id in RegisterId::ALL {
            let off = reg_offset(id);
            assert_eq!(off % 4, 0, "Register {id:?} offset {off:#x} is not 4-byte aligned");
            assert_eq!(RegisterId::from_aligned_offset(off), Some(id));
        }

        #[derive(Default)]
        struct SafetyMockBridge {
            voltage_limit: f32,
            current_limit: f32,
            vel_p: f32,
            control_mode: u8,
            min_pos: f32,
            max_pos: f32,
            stall_timeout_ms: u32,
            calib_cmd: u8,
            hw_err: u16,
            bus_v: f32,
            temp_c: f32,
        }

        impl RegisterBridge for SafetyMockBridge {
            fn read_reg(&self, id: RegisterId) -> Result<RegisterValue, barefoc_drivers::comms::registers::RegisterError> {
                match id {
                    RegisterId::VoltageLimit => Ok(RegisterValue::Float(self.voltage_limit)),
                    RegisterId::CurrentLimit => Ok(RegisterValue::Float(self.current_limit)),
                    RegisterId::VelPidP => Ok(RegisterValue::Float(self.vel_p)),
                    RegisterId::ControlMode => Ok(RegisterValue::U8(self.control_mode)),
                    RegisterId::MinPositionLimit => Ok(RegisterValue::Float(self.min_pos)),
                    RegisterId::MaxPositionLimit => Ok(RegisterValue::Float(self.max_pos)),
                    RegisterId::StallTimeoutMs => Ok(RegisterValue::U32(self.stall_timeout_ms)),
                    RegisterId::CalibCommand => Ok(RegisterValue::U8(self.calib_cmd)),
                    RegisterId::HardwareErrorStatus => Ok(RegisterValue::U16(self.hw_err)),
                    RegisterId::PresentBusVoltage => Ok(RegisterValue::Float(self.bus_v)),
                    RegisterId::PresentTemperature => Ok(RegisterValue::Float(self.temp_c)),
                    _ => Err(barefoc_drivers::comms::registers::RegisterError::UnknownRegister(id as u8)),
                }
            }

            fn write_reg(&mut self, id: RegisterId, value: RegisterValue) -> Result<(), barefoc_drivers::comms::registers::RegisterError> {
                match (id, value) {
                    (RegisterId::VoltageLimit, RegisterValue::Float(v)) => self.voltage_limit = v,
                    (RegisterId::CurrentLimit, RegisterValue::Float(v)) => self.current_limit = v,
                    (RegisterId::VelPidP, RegisterValue::Float(v)) => self.vel_p = v,
                    (RegisterId::ControlMode, RegisterValue::U8(v)) => self.control_mode = v,
                    (RegisterId::MinPositionLimit, RegisterValue::Float(v)) => self.min_pos = v,
                    (RegisterId::MaxPositionLimit, RegisterValue::Float(v)) => self.max_pos = v,
                    (RegisterId::StallTimeoutMs, RegisterValue::U32(v)) => self.stall_timeout_ms = v,
                    (RegisterId::CalibCommand, RegisterValue::U8(v)) => self.calib_cmd = v,
                    _ => return Err(barefoc_drivers::comms::registers::RegisterError::UnknownRegister(id as u8)),
                }
                Ok(())
            }
        }

        let b0 = SafetyMockBridge {
            hw_err: 0x0804,
            bus_v: 11.8,
            temp_c: 42.5,
            ..Default::default()
        };
        let b1 = SafetyMockBridge::default();

        let mut server = RegServer::new(multi_motor_profile([b0, b1]));
        refresh_telemetry_multi(server.map_mut());

        let mut bus = FakeI2cBus {
            server: &mut server,
        };
        let mut client = RegClientI2c::<_, 16>::new(&mut bus);

        // Write R/W safety & tuning registers over I2C
        client.write_register(DEV, reg_offset(RegisterId::VoltageLimit), &9.5f32.to_le_bytes()).unwrap();
        client.write_register(DEV, reg_offset(RegisterId::CurrentLimit), &1.8f32.to_le_bytes()).unwrap();
        client.write_register(DEV, reg_offset(RegisterId::VelPidP), &0.45f32.to_le_bytes()).unwrap();
        client.write_register(DEV, reg_offset(RegisterId::ControlMode), &[2]).unwrap();
        client.write_register(DEV, reg_offset(RegisterId::MinPositionLimit), &(-1.25f32).to_le_bytes()).unwrap();
        client.write_register(DEV, reg_offset(RegisterId::MaxPositionLimit), &1.75f32.to_le_bytes()).unwrap();
        client.write_register(DEV, reg_offset(RegisterId::StallTimeoutMs), &250u32.to_le_bytes()).unwrap();
        client.write_register(DEV, REG_CALIB_CMD, &[1]).unwrap();

        // Read back all written values
        let mut buf4 = [0u8; 4];
        let mut buf2 = [0u8; 2];
        let mut buf1 = [0u8; 1];

        client.read_register(DEV, reg_offset(RegisterId::VoltageLimit), 4, &mut buf4).unwrap();
        assert_eq!(f32::from_le_bytes(buf4), 9.5);
        client.read_register(DEV, reg_offset(RegisterId::CurrentLimit), 4, &mut buf4).unwrap();
        assert_eq!(f32::from_le_bytes(buf4), 1.8);
        client.read_register(DEV, reg_offset(RegisterId::VelPidP), 4, &mut buf4).unwrap();
        assert_eq!(f32::from_le_bytes(buf4), 0.45);
        client.read_register(DEV, reg_offset(RegisterId::ControlMode), 1, &mut buf1).unwrap();
        assert_eq!(buf1[0], 2);
        client.read_register(DEV, reg_offset(RegisterId::MinPositionLimit), 4, &mut buf4).unwrap();
        assert_eq!(f32::from_le_bytes(buf4), -1.25);
        client.read_register(DEV, reg_offset(RegisterId::MaxPositionLimit), 4, &mut buf4).unwrap();
        assert_eq!(f32::from_le_bytes(buf4), 1.75);
        client.read_register(DEV, reg_offset(RegisterId::StallTimeoutMs), 4, &mut buf4).unwrap();
        assert_eq!(u32::from_le_bytes(buf4), 250);
        client.read_register(DEV, REG_CALIB_CMD, 1, &mut buf1).unwrap();
        assert_eq!(buf1[0], 1);

        // Read RO telemetry registers
        client.read_register(DEV, reg_offset(RegisterId::HardwareErrorStatus), 2, &mut buf2).unwrap();
        assert_eq!(u16::from_le_bytes(buf2), 0x0804);
        client.read_register(DEV, reg_offset(RegisterId::PresentBusVoltage), 4, &mut buf4).unwrap();
        assert_eq!(f32::from_le_bytes(buf4), 11.8);
        client.read_register(DEV, reg_offset(RegisterId::PresentTemperature), 4, &mut buf4).unwrap();
        assert_eq!(f32::from_le_bytes(buf4), 42.5);

        // Runtime sensor & bus configuration registers (R/W) and SensorFaultStatus (RO)
        client.write_register(DEV, reg_offset(RegisterId::SensorBusFreqHz), &400_000u32.to_le_bytes()).unwrap();
        client.write_register(DEV, reg_offset(RegisterId::SensorRange), &[0]).unwrap();
        client.write_register(DEV, reg_offset(RegisterId::SensorAveraging), &[3]).unwrap();
        client.write_register(DEV, reg_offset(RegisterId::SensorCrcEnable), &[1]).unwrap();

        client.read_register(DEV, reg_offset(RegisterId::SensorBusFreqHz), 4, &mut buf4).unwrap();
        assert_eq!(u32::from_le_bytes(buf4), 400_000);
        client.read_register(DEV, reg_offset(RegisterId::SensorRange), 1, &mut buf1).unwrap();
        assert_eq!(buf1[0], 0);
        client.read_register(DEV, reg_offset(RegisterId::SensorAveraging), 1, &mut buf1).unwrap();
        assert_eq!(buf1[0], 3);
        client.read_register(DEV, reg_offset(RegisterId::SensorCrcEnable), 1, &mut buf1).unwrap();
        assert_eq!(buf1[0], 1);

        assert!(client.write_register(DEV, reg_offset(RegisterId::SensorFaultStatus), &[1]).is_err());
    }

    #[test]
    fn simplefoc_profile_seeds_rw_registers_from_bridge() {
        let clock = NullClock;
        let mut config = MotorConfig::new(&clock);
        config.target = 2.75;
        config.voltage_limit = 7.5;
        config.current_limit = 1.25;
        let map = simplefoc_profile(config);

        let mut buf = [0u8; 4];
        map.get(reg_offset(RegisterId::Target), &mut buf).unwrap();
        assert_eq!(f32::from_le_bytes(buf), 2.75);
        map.get(reg_offset(RegisterId::VoltageLimit), &mut buf).unwrap();
        assert_eq!(f32::from_le_bytes(buf), 7.5);
        map.get(reg_offset(RegisterId::CurrentLimit), &mut buf).unwrap();
        assert_eq!(f32::from_le_bytes(buf), 1.25);
    }

    #[test]
    fn multi_motor_profile_seeds_rw_on_init_and_motor_select() {
        let clock = NullClock;
        let mut c0 = MotorConfig::new(&clock);
        c0.target = 1.125;
        c0.voltage_limit = 6.0;
        c0.current_limit = 1.5;

        let mut c1 = MotorConfig::new(&clock);
        c1.target = -2.5;
        c1.voltage_limit = 9.0;
        c1.current_limit = 2.25;

        let mut map = multi_motor_profile([c0, c1]);

        let mut buf = [0u8; 4];
        map.get(reg_offset(RegisterId::Target), &mut buf).unwrap();
        assert_eq!(f32::from_le_bytes(buf), 1.125);
        map.get(reg_offset(RegisterId::VoltageLimit), &mut buf).unwrap();
        assert_eq!(f32::from_le_bytes(buf), 6.0);
        map.get(reg_offset(RegisterId::CurrentLimit), &mut buf).unwrap();
        assert_eq!(f32::from_le_bytes(buf), 1.5);

        // Switch to motor 1 via MotorSelect and refresh
        let sel_off = REG_MOTOR_SELECT.to_be_bytes();
        map.write_phase(&[sel_off[0], sel_off[1], 0x01]).unwrap();
        refresh_telemetry_multi(&mut map);

        map.get(reg_offset(RegisterId::Target), &mut buf).unwrap();
        assert_eq!(f32::from_le_bytes(buf), -2.5);
        map.get(reg_offset(RegisterId::VoltageLimit), &mut buf).unwrap();
        assert_eq!(f32::from_le_bytes(buf), 9.0);
        map.get(reg_offset(RegisterId::CurrentLimit), &mut buf).unwrap();
        assert_eq!(f32::from_le_bytes(buf), 2.25);
    }

    #[test]
    fn test_kinematics_register_bridge_roundtrip() {
        #[derive(Default)]
        struct KinBridge {
            tx_type: u8,
            gear_ratio: f32,
            diff_t11: f32,
            diff_t22: f32,
            l1: f32,
            l2_l4: [f32; 3],
            joint_offset_alpha: f32,
        }
        impl RegisterBridge for KinBridge {
            fn read_reg(&self, id: RegisterId) -> Result<RegisterValue, barefoc_drivers::comms::registers::RegisterError> {
                match id {
                    RegisterId::TransmissionType => Ok(RegisterValue::U8(self.tx_type)),
                    RegisterId::GearRatio => Ok(RegisterValue::Float(self.gear_ratio)),
                    RegisterId::DiffMatrix_T11 => Ok(RegisterValue::Float(self.diff_t11)),
                    RegisterId::DiffMatrix_T22 => Ok(RegisterValue::Float(self.diff_t22)),
                    RegisterId::LinkageL1 => Ok(RegisterValue::Float(self.l1)),
                    RegisterId::LinkageL2_L4 => Ok(RegisterValue::Float3(self.l2_l4)),
                    RegisterId::JointOffsetAlpha => Ok(RegisterValue::Float(self.joint_offset_alpha)),
                    _ => Err(barefoc_drivers::comms::registers::RegisterError::InvalidPayload),
                }
            }
            fn write_reg(&mut self, id: RegisterId, value: RegisterValue) -> Result<(), barefoc_drivers::comms::registers::RegisterError> {
                match (id, value) {
                    (RegisterId::TransmissionType, RegisterValue::U8(v)) => self.tx_type = v,
                    (RegisterId::GearRatio, RegisterValue::Float(v)) => self.gear_ratio = v,
                    (RegisterId::DiffMatrix_T11, RegisterValue::Float(v)) => self.diff_t11 = v,
                    (RegisterId::DiffMatrix_T22, RegisterValue::Float(v)) => self.diff_t22 = v,
                    (RegisterId::LinkageL1, RegisterValue::Float(v)) => self.l1 = v,
                    (RegisterId::LinkageL2_L4, RegisterValue::Float3(v)) => self.l2_l4 = v,
                    (RegisterId::JointOffsetAlpha, RegisterValue::Float(v)) => self.joint_offset_alpha = v,
                    _ => {}
                }
                Ok(())
            }
        }

        let mut map = multi_motor_profile([KinBridge::default(), KinBridge::default()]);
        let smap_off = REG_SENSOR_MAP.to_be_bytes();
        map.write_phase(&[smap_off[0], smap_off[1], 3, 2, 1, 0]).unwrap();
        let mut smap_buf = [0u8; 4];
        map.get(REG_SENSOR_MAP, &mut smap_buf).unwrap();
        assert_eq!(smap_buf, [3, 2, 1, 0]);

        let tx_off = reg_offset(RegisterId::TransmissionType).to_be_bytes();
        map.write_phase(&[tx_off[0], tx_off[1], 1]).unwrap();
        let gr_off = reg_offset(RegisterId::GearRatio).to_be_bytes();
        let mut gr_frame = [0u8; 6];
        gr_frame[0..2].copy_from_slice(&gr_off);
        gr_frame[2..6].copy_from_slice(&2.5f32.to_le_bytes());
        map.write_phase(&gr_frame).unwrap();

        let t11_off = reg_offset(RegisterId::DiffMatrix_T11).to_be_bytes();
        let mut t11_frame = [0u8; 6];
        t11_frame[0..2].copy_from_slice(&t11_off);
        t11_frame[2..6].copy_from_slice(&0.5f32.to_le_bytes());
        map.write_phase(&t11_frame).unwrap();

        let l24_off = reg_offset(RegisterId::LinkageL2_L4).to_be_bytes();
        let mut l24_frame = [0u8; 14];
        l24_frame[0..2].copy_from_slice(&l24_off);
        l24_frame[2..6].copy_from_slice(&30.0f32.to_le_bytes());
        l24_frame[6..10].copy_from_slice(&90.0f32.to_le_bytes());
        l24_frame[10..14].copy_from_slice(&40.0f32.to_le_bytes());
        map.write_phase(&l24_frame).unwrap();

        assert_eq!(map.handler().motors[0].tx_type, 1);
        assert_eq!(map.handler().motors[0].gear_ratio, 2.5);
        assert_eq!(map.handler().motors[0].diff_t11, 0.5);
        assert_eq!(map.handler().motors[0].l2_l4, [30.0, 90.0, 40.0]);
    }
}
