//! # Universal Control Table Profile for MultiBus
//!
//! Exposes the standardized 512-byte [`ControlTable`] over MultiBus at canonical
//! 16-bit offsets. Register pointers are two-byte big-endian addresses (design D5
//! of `extended-telemetry-controls`), so telemetry at `0x180+` is reachable without
//! modulo-256 aliases into the motor bank. EEPROM registers (`0x00..0xDF`) are
//! persistent configuration; RAM registers (`0xE0..0x1FF`) are volatile targets
//! and telemetry.
//!
//! Wires write hooks for:
//! - `SaveConfigCommand` (0x124): writes of `0x55` invoke `ConfigStorage::save_config`
//!   and self-clear.
//! - `RebootCommand` (0x123): writes of `0xF0` invoke `BootloaderHandoff::enter_bootloader`.
//! - `WatchdogTimeoutMs` (0x1C0): writes feed the watchdog, resetting the
//!   remaining-time countdown at `0x1C2`.
//! - `GoalTorqueFF` (0x1AC): writes publish the torque-feedforward setpoint into
//!   the live [`ControlTable`].

use barefoc::control_table::*;
use barefoc::traits::bootloader::BootloaderHandoff;
use barefoc::traits::storage::ConfigStorage;

use crate::table::{HookResult, RegEntry, RegMap};

/// Application context holding the active [`ControlTable`], optional Flash persistence
/// backend `S`, and optional bootloader hand-off driver `B`.
pub struct ControlTableContext<S = (), B = ()> {
    /// Active in-memory Control Table.
    pub table: ControlTable,
    /// Optional storage persistence backend.
    pub storage: Option<S>,
    /// Optional Katapult bootloader hand-off mechanism.
    pub bootloader: Option<B>,
    /// Optional runtime alignment handler: receives axis index.
    pub align_handler: Option<fn(&mut ControlTable, usize)>,
}

impl<S, B> ControlTableContext<S, B> {
    /// Create a new context with a Control Table and optional drivers.
    pub fn new(table: ControlTable, storage: Option<S>, bootloader: Option<B>) -> Self {
        Self {
            table,
            storage,
            bootloader,
            align_handler: None,
        }
    }

    /// Attach a runtime alignment handler.
    pub fn with_align_handler(mut self, handler: fn(&mut ControlTable, usize)) -> Self {
        self.align_handler = Some(handler);
        self
    }
}

fn save_hook<S: ConfigStorage, B>(
    ctx: &mut ControlTableContext<S, B>,
    _offset: u16,
    bytes: &[u8],
) -> HookResult {
    if bytes.is_empty() {
        return HookResult::Applied;
    }
    match bytes[0] {
        CMD_SAVE_CONFIG => {
            if let Some(ref mut storage) = ctx.storage {
                let _ = storage.save_config(&ctx.table);
            }
            HookResult::SelfClear
        }
        CMD_FACTORY_RESET => {
            if let Some(ref mut storage) = ctx.storage {
                let _ = storage.factory_reset(&mut ctx.table);
            }
            HookResult::SelfClear
        }
        _ => HookResult::Applied,
    }
}

fn reboot_hook<S, B: BootloaderHandoff>(
    ctx: &mut ControlTableContext<S, B>,
    _offset: u16,
    bytes: &[u8],
) -> HookResult {
    if bytes.is_empty() {
        return HookResult::Applied;
    }
    if bytes[0] == CMD_REBOOT_BOOTLOADER {
        if let Some(ref mut bootloader) = ctx.bootloader {
            bootloader.enter_bootloader();
        }
    }
    HookResult::Applied
}

/// Write hook for `REG_WATCHDOG_TIMEOUT_MS` (`0x1C0`): publishes the timeout to
/// the live [`ControlTable`] and re-arms the countdown (a bus keepalive).
fn watchdog_hook<S, B>(ctx: &mut ControlTableContext<S, B>, _offset: u16, bytes: &[u8]) -> HookResult {
    if bytes.len() >= 2 {
        let timeout = u16::from_le_bytes([bytes[0], bytes[1]]);
        ctx.table.set_watchdog_timeout_ms(timeout);
        ctx.table.feed_watchdog();
    }
    HookResult::Applied
}

/// Write hook for `REG_GOAL_TORQUE_FF` (`0x1AC`): publishes the feedforward
/// torque setpoint into the live [`ControlTable`] for the current loop.
fn torque_ff_hook<S, B>(ctx: &mut ControlTableContext<S, B>, _offset: u16, bytes: &[u8]) -> HookResult {
    if bytes.len() >= 4 {
        let value = f32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);
        ctx.table.set_goal_torque_ff(value);
    }
    HookResult::Applied
}

fn calib_hook<S: ConfigStorage, B>(
    ctx: &mut ControlTableContext<S, B>,
    _offset: u16,
    bytes: &[u8],
) -> HookResult {
    if bytes.is_empty() {
        return HookResult::Applied;
    }
    match bytes[0] {
        CMD_CALIB_ALIGN => {
            if let Some(handler) = ctx.align_handler {
                handler(&mut ctx.table, 0);
            }
            if let Some(ref mut storage) = ctx.storage {
                let _ = storage.save_config(&ctx.table);
            }
            HookResult::SelfClear
        }
        CMD_CALIB_SWEEP => {
            HookResult::SelfClear
        }
        CMD_CALIB_SAVE => {
            if let Some(ref mut storage) = ctx.storage {
                let _ = storage.save_config(&ctx.table);
            }
            HookResult::SelfClear
        }
        CMD_CALIB_SET_ORIGIN => {
            let pos = ctx.table.present_position();
            ctx.table.set_homing_offset(pos);
            ctx.table.set_motor_homing_offset(0, pos);
            if let Some(ref mut storage) = ctx.storage {
                let _ = storage.save_config(&ctx.table);
            }
            HookResult::SelfClear
        }
        _ => HookResult::Applied,
    }
}

/// Build a MultiBus [`RegMap`] from a [`ControlTableContext`].
pub fn control_table_profile<S: ConfigStorage, B: BootloaderHandoff>(
    ctx: ControlTableContext<S, B>,
) -> RegMap<ControlTableContext<S, B>, CONTROL_TABLE_SIZE, 128> {
    let mut map: RegMap<ControlTableContext<S, B>, CONTROL_TABLE_SIZE, 128> = RegMap::new(ctx);

    // -----------------------------------------------------------------------
    // EEPROM Area (0x00..0x5F)
    // -----------------------------------------------------------------------
    let _ = map.add_register(RegEntry::ro(REG_MODEL_NUMBER as u16, 2));
    let _ = map.add_register(RegEntry::ro(REG_FIRMWARE_VERSION as u16, 1));
    let _ = map.add_register(RegEntry::ro(REG_PROTOCOL_VERSION as u16, 1));
    let _ = map.add_register(RegEntry::rw(REG_NODE_ID as u16, 1));
    let _ = map.add_register(RegEntry::rw(REG_OPERATING_MODE as u16, 1));
    let _ = map.add_register(RegEntry::rw(REG_SECONDARY_ID as u16, 1));
    let _ = map.add_register(RegEntry::rw(REG_TOPOLOGY_PRESET as u16, 1));
    let _ = map.add_register(RegEntry::rw(REG_COUPLING_MASK as u16, 1));

    let _ = map.add_register(RegEntry::rw(REG_HOMING_OFFSET as u16, 4));
    let _ = map.add_register(RegEntry::rw(REG_CURRENT_LIMIT as u16, 4));
    let _ = map.add_register(RegEntry::rw(REG_ACCELERATION_LIMIT as u16, 4));
    let _ = map.add_register(RegEntry::rw(REG_VELOCITY_LIMIT as u16, 4));
    let _ = map.add_register(RegEntry::rw(REG_MIN_POSITION_LIMIT as u16, 4));
    let _ = map.add_register(RegEntry::rw(REG_MAX_POSITION_LIMIT as u16, 4));
    let _ = map.add_register(RegEntry::rw(REG_VOLTAGE_LIMIT as u16, 4));
    let _ = map.add_register(RegEntry::rw(REG_TEMPERATURE_LIMIT as u16, 4));
    let _ = map.add_register(RegEntry::rw(REG_MIN_BUS_VOLTAGE as u16, 4));
    let _ = map.add_register(RegEntry::rw(REG_MAX_BUS_VOLTAGE as u16, 4));

    let _ = map.add_register(RegEntry::rw(REG_POS_P_GAIN as u16, 4));
    let _ = map.add_register(RegEntry::rw(REG_POS_I_GAIN as u16, 4));
    let _ = map.add_register(RegEntry::rw(REG_POS_D_GAIN as u16, 4));
    let _ = map.add_register(RegEntry::rw(REG_VEL_P_GAIN as u16, 4));
    let _ = map.add_register(RegEntry::rw(REG_VEL_I_GAIN as u16, 4));
    let _ = map.add_register(RegEntry::rw(REG_VEL_D_GAIN as u16, 4));
    let _ = map.add_register(RegEntry::rw(REG_CURRENT_P_GAIN as u16, 4));
    let _ = map.add_register(RegEntry::rw(REG_CURRENT_I_GAIN as u16, 4));
    let _ = map.add_register(RegEntry::rw(REG_CURRENT_D_GAIN as u16, 4));

    // -----------------------------------------------------------------------
    // RAM Area (0x60..0xAF)
    // -----------------------------------------------------------------------
    let _ = map.add_register(RegEntry::rw(REG_TORQUE_ENABLE as u16, 1));
    let _ = map.add_register(RegEntry::rw(REG_LED as u16, 1));
    let _ = map.add_register(RegEntry::rw(REG_STATUS_RETURN_LEVEL as u16, 1));

    // Reboot hook (0x63)
    let _ = map.add_register(RegEntry::rw_hooked(
        REG_REBOOT_CMD as u16,
        1,
        reboot_hook::<S, B>,
    ));

    // Save config hook (0x64)
    let _ = map.add_register(RegEntry::rw_hooked(
        REG_SAVE_CONFIG_CMD as u16,
        1,
        save_hook::<S, B>,
    ));

    // Calibration command hook (0xB0)
    let _ = map.add_register(RegEntry::rw_hooked(
        REG_CALIB_CMD as u16,
        1,
        calib_hook::<S, B>,
    ));

    let _ = map.add_register(RegEntry::rw(REG_PROFILE_VELOCITY as u16, 4));
    let _ = map.add_register(RegEntry::rw(REG_PROFILE_ACCELERATION as u16, 4));
    let _ = map.add_register(RegEntry::rw(REG_GOAL_POSITION as u16, 4));
    let _ = map.add_register(RegEntry::rw(REG_GOAL_VELOCITY as u16, 4));
    let _ = map.add_register(RegEntry::rw(REG_GOAL_CURRENT as u16, 4));
    let _ = map.add_register(RegEntry::rw(REG_GOAL_VOLTAGE as u16, 4));

    // Telemetry burst read block (0x80..0xA0, 32 bytes)
    let _ = map.add_register(RegEntry::ro(REG_HARDWARE_ERROR_STATUS as u16, 2));
    let _ = map.add_register(RegEntry::ro(REG_PRESENT_TEMPERATURE as u16, 2));

    // Multi-zone thermal telemetry & limits. The MOSFET zone aliases the two
    // legacy registers above (0x142 / 0x100) and is intentionally not
    // re-registered: RegMap rejects overlapping descriptors. With the 16-bit
    // pointer (design D5) the remaining zones use their canonical 16-bit
    // offsets directly (0x16A, 0x16E, 0x172, 0x176).
    let _ = map.add_register(RegEntry::ro(REG_PRESENT_TEMP_MCU as u16, 4));
    let _ = map.add_register(RegEntry::ro(REG_PRESENT_TEMP_MOTOR as u16, 4));
    let _ = map.add_register(RegEntry::rw(REG_TEMP_LIMIT_MCU as u16, 4));
    let _ = map.add_register(RegEntry::rw(REG_TEMP_LIMIT_MOTOR as u16, 4));

    let _ = map.add_register(RegEntry::ro(REG_PRESENT_POSITION as u16, 4));
    let _ = map.add_register(RegEntry::ro(REG_PRESENT_VELOCITY as u16, 4));
    let _ = map.add_register(RegEntry::ro(REG_PRESENT_CURRENT as u16, 4));
    let _ = map.add_register(RegEntry::ro(REG_PRESENT_VOLTAGE as u16, 4));
    let _ = map.add_register(RegEntry::ro(REG_PRESENT_BUS_VOLTAGE as u16, 4));
    let _ = map.add_register(RegEntry::ro(REG_FILTERED_CURRENT as u16, 4));
    let _ = map.add_register(RegEntry::ro(REG_TRAJECTORY_POSITION as u16, 4));

    let _ = map.add_register(RegEntry::ro(REG_MOVING as u16, 1));
    let _ = map.add_register(RegEntry::ro(REG_MULTI_TURN_REVOLUTIONS as u16, 4));
    let _ = map.add_register(RegEntry::ro(REG_MULTI_TURN_ANGLE as u16, 4));

    // -----------------------------------------------------------------------
    // Capability bitfield at 0x1F8 (RAM tail, after the fault FIFO) and extended telemetry tail
    // (0x180..0x1FF). All offsets are canonical 16-bit addresses; the 2-byte
    // big-endian pointer makes them addressable over the bus.
    // -----------------------------------------------------------------------
    let _ = map.add_register(RegEntry::ro(REG_HARDWARE_CAPABILITIES as u16, 4));

    // Electrical power & energy telemetry.
    let _ = map.add_register(RegEntry::ro(REG_PRESENT_BUS_CURRENT as u16, 4));
    let _ = map.add_register(RegEntry::ro(REG_PRESENT_BUS_POWER as u16, 4));
    let _ = map.add_register(RegEntry::ro(REG_ACCUMULATED_ENERGY as u16, 4));
    let _ = map.add_register(RegEntry::ro(REG_FOC_IQ_MEASURED as u16, 4));
    let _ = map.add_register(RegEntry::ro(REG_FOC_ID_MEASURED as u16, 4));
    let _ = map.add_register(RegEntry::ro(REG_MODULATION_INDEX as u16, 4));
    let _ = map.add_register(RegEntry::ro(REG_BRAKE_CHOPPER_DUTY as u16, 1));

    // Robotics dynamics, torque feedforward & virtual impedance.
    let _ = map.add_register(RegEntry::ro(REG_PRESENT_TORQUE as u16, 4));
    let _ = map.add_register(RegEntry::ro(REG_PRESENT_ACCELERATION as u16, 4));
    let _ = map.add_register(RegEntry::ro(REG_TRACKING_ERROR as u16, 4));
    let _ = map.add_register(RegEntry::ro(REG_GEAR_DEFLECTION as u16, 4));
    let _ = map.add_register(RegEntry::rw_hooked(
        REG_GOAL_TORQUE_FF as u16,
        4,
        torque_ff_hook::<S, B>,
    ));
    let _ = map.add_register(RegEntry::rw(REG_IMPEDANCE_KP as u16, 4));
    let _ = map.add_register(RegEntry::rw(REG_IMPEDANCE_KD as u16, 4));

    // Sensor integrity diagnostics.
    let _ = map.add_register(RegEntry::ro(REG_MAG_FIELD_MAGNITUDE as u16, 4));
    let _ = map.add_register(RegEntry::ro(REG_MAG_STATUS_FLAGS as u16, 1));
    let _ = map.add_register(RegEntry::ro(REG_SENSOR_CRC_ERRORS as u16, 2));

    // Communication watchdog with write-hook keepalive.
    let _ = map.add_register(RegEntry::rw_hooked(
        REG_WATCHDOG_TIMEOUT_MS as u16,
        2,
        watchdog_hook::<S, B>,
    ));
    let _ = map.add_register(RegEntry::ro(REG_WATCHDOG_TIME_LEFT_MS as u16, 2));

    // Trajectory state & position deadband.
    let _ = map.add_register(RegEntry::ro(REG_TRAJECTORY_STATUS as u16, 1));
    let _ = map.add_register(RegEntry::ro(REG_IN_POSITION_BAND as u16, 4));
    let _ = map.add_register(RegEntry::rw(REG_POSITION_DEADBAND as u16, 4));

    // RTOS / execution profiling.
    let _ = map.add_register(RegEntry::ro(REG_LOOP_TIME_US as u16, 2));
    let _ = map.add_register(RegEntry::ro(REG_MAX_LOOP_JITTER_US as u16, 2));
    let _ = map.add_register(RegEntry::ro(REG_CPU_LOAD_PCT as u16, 1));

    // Extended 32-bit error flags (shadow of the legacy 0x140 word) and odometer.
    let _ = map.add_register(RegEntry::ro(REG_HARDWARE_ERROR_STATUS_EXT as u16, 4));
    let _ = map.add_register(RegEntry::ro(REG_TOTAL_OPERATING_HOURS as u16, 4));
    let _ = map.add_register(RegEntry::ro(REG_TOTAL_REVOLUTIONS as u16, 8));
    let _ = map.add_register(RegEntry::ro(REG_FAULT_HISTORY as u16, 16));

    // Seed map bank from initial context table
    sync_all_to_map(&mut map);

    map
}

/// Synchronize all registers from `handler().table` into the MultiBus bank.
pub fn sync_all_to_map<S, B>(map: &mut RegMap<ControlTableContext<S, B>, CONTROL_TABLE_SIZE, 128>) {
    let t = map.handler().table;
    let _ = map.set(REG_MODEL_NUMBER as u16, &t.read_u16(REG_MODEL_NUMBER).to_le_bytes());
    let _ = map.set(REG_FIRMWARE_VERSION as u16, &[t.read_u8(REG_FIRMWARE_VERSION)]);
    let _ = map.set(REG_PROTOCOL_VERSION as u16, &[t.read_u8(REG_PROTOCOL_VERSION)]);
    let _ = map.set(REG_NODE_ID as u16, &[t.node_id()]);
    let _ = map.set(REG_OPERATING_MODE as u16, &[t.read_u8(REG_OPERATING_MODE)]);
    let _ = map.set(REG_SECONDARY_ID as u16, &[t.read_u8(REG_SECONDARY_ID)]);
    let _ = map.set(REG_TOPOLOGY_PRESET as u16, &[t.read_u8(REG_TOPOLOGY_PRESET)]);
    let _ = map.set(REG_COUPLING_MASK as u16, &[t.coupling_mask()]);

    let _ = map.set(REG_HOMING_OFFSET as u16, &t.read_f32(REG_HOMING_OFFSET).to_le_bytes());
    let _ = map.set(REG_CURRENT_LIMIT as u16, &t.current_limit().to_le_bytes());
    let _ = map.set(REG_ACCELERATION_LIMIT as u16, &t.acceleration_limit().to_le_bytes());
    let _ = map.set(REG_VELOCITY_LIMIT as u16, &t.velocity_limit().to_le_bytes());
    let _ = map.set(REG_MIN_POSITION_LIMIT as u16, &t.min_position_limit().to_le_bytes());
    let _ = map.set(REG_MAX_POSITION_LIMIT as u16, &t.max_position_limit().to_le_bytes());
    let _ = map.set(REG_VOLTAGE_LIMIT as u16, &t.voltage_limit().to_le_bytes());
    let _ = map.set(REG_TEMPERATURE_LIMIT as u16, &t.temperature_limit().to_le_bytes());
    let _ = map.set(REG_MIN_BUS_VOLTAGE as u16, &t.min_bus_voltage().to_le_bytes());
    let _ = map.set(REG_MAX_BUS_VOLTAGE as u16, &t.max_bus_voltage().to_le_bytes());

    let _ = map.set(REG_POS_P_GAIN as u16, &t.read_f32(REG_POS_P_GAIN).to_le_bytes());
    let _ = map.set(REG_POS_I_GAIN as u16, &t.read_f32(REG_POS_I_GAIN).to_le_bytes());
    let _ = map.set(REG_POS_D_GAIN as u16, &t.read_f32(REG_POS_D_GAIN).to_le_bytes());
    let _ = map.set(REG_VEL_P_GAIN as u16, &t.read_f32(REG_VEL_P_GAIN).to_le_bytes());
    let _ = map.set(REG_VEL_I_GAIN as u16, &t.read_f32(REG_VEL_I_GAIN).to_le_bytes());
    let _ = map.set(REG_VEL_D_GAIN as u16, &t.read_f32(REG_VEL_D_GAIN).to_le_bytes());
    let _ = map.set(REG_CURRENT_P_GAIN as u16, &t.read_f32(REG_CURRENT_P_GAIN).to_le_bytes());
    let _ = map.set(REG_CURRENT_I_GAIN as u16, &t.read_f32(REG_CURRENT_I_GAIN).to_le_bytes());
    let _ = map.set(REG_CURRENT_D_GAIN as u16, &t.read_f32(REG_CURRENT_D_GAIN).to_le_bytes());

    let _ = map.set(REG_TORQUE_ENABLE as u16, &[if t.torque_enable() { 1 } else { 0 }]);
    let _ = map.set(REG_LED as u16, &[t.read_u8(REG_LED)]);
    let _ = map.set(REG_STATUS_RETURN_LEVEL as u16, &[t.read_u8(REG_STATUS_RETURN_LEVEL)]);
    let _ = map.set(REG_PROFILE_VELOCITY as u16, &t.read_f32(REG_PROFILE_VELOCITY).to_le_bytes());
    let _ = map.set(REG_PROFILE_ACCELERATION as u16, &t.read_f32(REG_PROFILE_ACCELERATION).to_le_bytes());
    let _ = map.set(REG_GOAL_POSITION as u16, &t.goal_position().to_le_bytes());
    let _ = map.set(REG_GOAL_VELOCITY as u16, &t.goal_velocity().to_le_bytes());
    let _ = map.set(REG_GOAL_CURRENT as u16, &t.goal_current().to_le_bytes());
    let _ = map.set(REG_GOAL_VOLTAGE as u16, &t.goal_voltage().to_le_bytes());

    let _ = map.set(REG_HARDWARE_ERROR_STATUS as u16, &t.hardware_error_status().to_le_bytes());
    let _ = map.set(REG_PRESENT_TEMPERATURE as u16, &t.read_u16(REG_PRESENT_TEMPERATURE).to_le_bytes());
    let _ = map.set(REG_PRESENT_TEMP_MCU as u16, &t.present_temp_mcu().to_le_bytes());
    let _ = map.set(REG_PRESENT_TEMP_MOTOR as u16, &t.present_temp_motor().to_le_bytes());
    let _ = map.set(REG_TEMP_LIMIT_MCU as u16, &t.temp_limit_mcu().to_le_bytes());
    let _ = map.set(REG_TEMP_LIMIT_MOTOR as u16, &t.temp_limit_motor().to_le_bytes());
    let _ = map.set(REG_PRESENT_POSITION as u16, &t.read_f32(REG_PRESENT_POSITION).to_le_bytes());
    let _ = map.set(REG_PRESENT_VELOCITY as u16, &t.read_f32(REG_PRESENT_VELOCITY).to_le_bytes());
    let _ = map.set(REG_PRESENT_CURRENT as u16, &t.read_f32(REG_PRESENT_CURRENT).to_le_bytes());
    let _ = map.set(REG_PRESENT_VOLTAGE as u16, &t.read_f32(REG_PRESENT_VOLTAGE).to_le_bytes());
    let _ = map.set(REG_PRESENT_BUS_VOLTAGE as u16, &t.read_f32(REG_PRESENT_BUS_VOLTAGE).to_le_bytes());
    let _ = map.set(REG_FILTERED_CURRENT as u16, &t.read_f32(REG_FILTERED_CURRENT).to_le_bytes());
    let _ = map.set(REG_TRAJECTORY_POSITION as u16, &t.read_f32(REG_TRAJECTORY_POSITION).to_le_bytes());
    let _ = map.set(REG_MOVING as u16, &[t.read_u8(REG_MOVING)]);
    let _ = map.set(REG_MULTI_TURN_REVOLUTIONS as u16, &t.read_i32(REG_MULTI_TURN_REVOLUTIONS).to_le_bytes());
    let _ = map.set(REG_MULTI_TURN_ANGLE as u16, &t.read_f32(REG_MULTI_TURN_ANGLE).to_le_bytes());

    // Extended telemetry tail. Published by firmware; RO from the bus.
    let _ = map.set(REG_HARDWARE_CAPABILITIES as u16, &t.hardware_capabilities().to_le_bytes());
    let _ = map.set(REG_PRESENT_BUS_CURRENT as u16, &t.present_bus_current().to_le_bytes());
    let _ = map.set(REG_PRESENT_BUS_POWER as u16, &t.present_bus_power().to_le_bytes());
    let _ = map.set(REG_ACCUMULATED_ENERGY as u16, &t.accumulated_energy().to_le_bytes());
    let _ = map.set(REG_FOC_IQ_MEASURED as u16, &t.foc_iq_measured().to_le_bytes());
    let _ = map.set(REG_FOC_ID_MEASURED as u16, &t.foc_id_measured().to_le_bytes());
    let _ = map.set(REG_MODULATION_INDEX as u16, &t.modulation_index().to_le_bytes());
    let _ = map.set(REG_BRAKE_CHOPPER_DUTY as u16, &[t.brake_chopper_duty()]);
    let _ = map.set(REG_PRESENT_TORQUE as u16, &t.present_torque().to_le_bytes());
    let _ = map.set(REG_PRESENT_ACCELERATION as u16, &t.present_acceleration().to_le_bytes());
    let _ = map.set(REG_TRACKING_ERROR as u16, &t.tracking_error().to_le_bytes());
    let _ = map.set(REG_GEAR_DEFLECTION as u16, &t.gear_deflection().to_le_bytes());
    let _ = map.set(REG_GOAL_TORQUE_FF as u16, &t.goal_torque_ff().to_le_bytes());
    let _ = map.set(REG_IMPEDANCE_KP as u16, &t.impedance_kp().to_le_bytes());
    let _ = map.set(REG_IMPEDANCE_KD as u16, &t.impedance_kd().to_le_bytes());
    let _ = map.set(REG_MAG_FIELD_MAGNITUDE as u16, &t.mag_field_magnitude().to_le_bytes());
    let _ = map.set(REG_MAG_STATUS_FLAGS as u16, &[t.mag_status_flags()]);
    let _ = map.set(REG_SENSOR_CRC_ERRORS as u16, &t.sensor_crc_errors().to_le_bytes());
    let _ = map.set(REG_WATCHDOG_TIMEOUT_MS as u16, &t.watchdog_timeout_ms().to_le_bytes());
    let _ = map.set(REG_WATCHDOG_TIME_LEFT_MS as u16, &t.watchdog_time_left_ms().to_le_bytes());
    let _ = map.set(REG_TRAJECTORY_STATUS as u16, &[t.trajectory_status()]);
    let _ = map.set(REG_IN_POSITION_BAND as u16, &t.in_position_band().to_le_bytes());
    let _ = map.set(REG_POSITION_DEADBAND as u16, &t.position_deadband().to_le_bytes());
    let _ = map.set(REG_LOOP_TIME_US as u16, &t.loop_time_us().to_le_bytes());
    let _ = map.set(REG_MAX_LOOP_JITTER_US as u16, &t.max_loop_jitter_us().to_le_bytes());
    let _ = map.set(REG_CPU_LOAD_PCT as u16, &[t.cpu_load_pct()]);
    let _ = map.set(REG_HARDWARE_ERROR_STATUS_EXT as u16, &t.hardware_error_status_ext().to_le_bytes());
    let _ = map.set(REG_TOTAL_OPERATING_HOURS as u16, &t.total_operating_hours().to_le_bytes());
    let _ = map.set(REG_TOTAL_REVOLUTIONS as u16, &t.total_revolutions().to_le_bytes());
    let mut fault_bytes = [0u8; FAULT_HISTORY_SLOTS * FAULT_RECORD_SIZE];
    for i in 0..FAULT_HISTORY_SLOTS {
        let rec = t.fault_record(i).unwrap_or(barefoc::control_table::FaultRecord {
            code: 0,
            zone: 0,
            timestamp_ms: 0,
        });
        let bytes = rec.to_bytes();
        let start = i * FAULT_RECORD_SIZE;
        fault_bytes[start..start + FAULT_RECORD_SIZE].copy_from_slice(&bytes);
    }
    let _ = map.set(REG_FAULT_HISTORY as u16, &fault_bytes);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::client::RegClientI2c;
    use crate::transport::{RegServer, TargetTransport};
    use embedded_hal::i2c as eh_i2c;
    use embedded_hal::i2c::I2c;

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
            panic!("ENTER_BOOTLOADER_TRIGGERED");
        }
    }

    #[test]
    fn test_telemetry_burst_read_32_bytes() {
        let mut table = ControlTable::new();
        table.set_hardware_error_status(0x0001);
        table.write_f32(REG_PRESENT_POSITION, 3.14159);
        table.write_f32(REG_PRESENT_VELOCITY, 10.0);
        table.write_f32(REG_PRESENT_CURRENT, 0.75);
        table.write_f32(REG_PRESENT_VOLTAGE, 4.2);
        table.write_f32(REG_PRESENT_BUS_VOLTAGE, 5.05);

        let storage = MockStorage { save_count: 0 };
        let bootloader = MockBootloader { entered: false };
        let ctx = ControlTableContext::new(table, Some(storage), Some(bootloader));
        let mut map = control_table_profile(ctx);

        // Latch pointer to REG_HARDWARE_ERROR_STATUS
        map.write_phase(&[(REG_HARDWARE_ERROR_STATUS >> 8) as u8, REG_HARDWARE_ERROR_STATUS as u8]).unwrap();

        // Read burst of 32 bytes
        let mut burst_buf = [0u8; 32];
        let n = map.read_phase(32, &mut burst_buf).unwrap();
        assert_eq!(n, 32);

        // Hardware error status at offset 0 in burst
        let err_status = u16::from_le_bytes([burst_buf[0], burst_buf[1]]);
        assert_eq!(err_status, 0x0001);

        // Present position at offset 4 in burst
        let pos = f32::from_le_bytes([burst_buf[4], burst_buf[5], burst_buf[6], burst_buf[7]]);
        assert!((pos - 3.14159).abs() < 1e-4);

        // Present bus voltage at offset 20 in burst
        let vbus = f32::from_le_bytes([burst_buf[20], burst_buf[21], burst_buf[22], burst_buf[23]]);
        assert!((vbus - 5.05).abs() < 1e-4);
    }

    #[test]
    fn test_save_config_hook() {
        let table = ControlTable::new();
        let storage = MockStorage { save_count: 0 };
        let ctx = ControlTableContext::new(table, Some(storage), None::<MockBootloader>);
        let mut map = control_table_profile(ctx);

        assert_eq!(map.handler().storage.as_ref().unwrap().save_count, 0);

        // Write 0x55 to SaveConfigCommand
        map.write_phase(&[(REG_SAVE_CONFIG_CMD >> 8) as u8, REG_SAVE_CONFIG_CMD as u8, 0x55]).unwrap();

        // Verify save_config was called
        assert_eq!(map.handler().storage.as_ref().unwrap().save_count, 1);

        // Register must self-clear
        let mut val = [0u8; 1];
        let _ = map.get(REG_SAVE_CONFIG_CMD as u16, &mut val).unwrap();
        assert_eq!(val[0], 0x00);
    }

    fn dummy_align_handler(table: &mut ControlTable, _axis: usize) {
        table.set_motor_zero_electric_angle(0, 2.0);
        table.set_motor_sensor_direction(0, 1);
    }

    #[test]
    fn test_calib_cmd_align() {
        let table = ControlTable::new();
        let storage = MockStorage { save_count: 0 };
        let ctx = ControlTableContext::new(table, Some(storage), None::<MockBootloader>)
            .with_align_handler(dummy_align_handler);
        let mut map = control_table_profile(ctx);

        // Write 0x01 (CMD_CALIB_ALIGN) to CALIB_CMD (0xB0)
        map.write_phase(&[(REG_CALIB_CMD >> 8) as u8, REG_CALIB_CMD as u8, CMD_CALIB_ALIGN]).unwrap();

        assert_eq!(map.handler().table.motor_zero_electric_angle(0), 2.0);
        assert_eq!(map.handler().table.motor_sensor_direction(0), 1);
        assert_eq!(map.handler().storage.as_ref().unwrap().save_count, 1);

        // Register must self-clear
        let mut val = [0u8; 1];
        let _ = map.get(REG_CALIB_CMD as u16, &mut val).unwrap();
        assert_eq!(val[0], 0x00);
    }

    #[test]
    fn test_calib_cmd_set_origin() {
        let mut table = ControlTable::new();
        table.set_present_position(1.5);
        let storage = MockStorage { save_count: 0 };
        let ctx = ControlTableContext::new(table, Some(storage), None::<MockBootloader>);
        let mut map = control_table_profile(ctx);

        // Write 0x04 (CMD_CALIB_SET_ORIGIN) to CALIB_CMD (0xB0)
        map.write_phase(&[(REG_CALIB_CMD >> 8) as u8, REG_CALIB_CMD as u8, CMD_CALIB_SET_ORIGIN]).unwrap();

        assert_eq!(map.handler().table.homing_offset(), 1.5);
        assert_eq!(map.handler().table.motor_homing_offset(0), 1.5);
        assert_eq!(map.handler().storage.as_ref().unwrap().save_count, 1);

        let mut val = [0u8; 1];
        let _ = map.get(REG_CALIB_CMD as u16, &mut val).unwrap();
        assert_eq!(val[0], 0x00);
    }

    #[test]
    fn test_calib_cmd_save_and_sweep() {
        let table = ControlTable::new();
        let storage = MockStorage { save_count: 0 };
        let ctx = ControlTableContext::new(table, Some(storage), None::<MockBootloader>);
        let mut map = control_table_profile(ctx);

        // Write 0x03 (CMD_CALIB_SAVE)
        map.write_phase(&[(REG_CALIB_CMD >> 8) as u8, REG_CALIB_CMD as u8, CMD_CALIB_SAVE]).unwrap();
        assert_eq!(map.handler().storage.as_ref().unwrap().save_count, 1);

        // Write 0x02 (CMD_CALIB_SWEEP)
        map.write_phase(&[(REG_CALIB_CMD >> 8) as u8, REG_CALIB_CMD as u8, CMD_CALIB_SWEEP]).unwrap();
        let mut val = [0u8; 1];
        let _ = map.get(REG_CALIB_CMD as u16, &mut val).unwrap();
        assert_eq!(val[0], 0x00);
    }

    #[test]
    fn test_thermal_zone_registers_roundtrip() {
        let mut table = ControlTable::new();
        table.set_present_temp_mosfet(55.0);
        table.set_present_temp_mcu(61.5);
        table.set_present_temp_motor(72.25);
        table.set_temp_limit_mcu(85.0);
        table.set_temp_limit_motor(90.0);

        let ctx = ControlTableContext::new(table, None::<MockStorage>, None::<MockBootloader>);
        let mut map = control_table_profile(ctx);

        // The 16-bit pointer (design D5) exposes canonical offsets directly:
        // 0x16A, 0x16E, 0x172 and 0x176 are used verbatim, with no modulo alias.
        //
        // MOSFET telemetry is the legacy alias at 0x142
        // and is already registered via REG_PRESENT_TEMPERATURE. The MOSFET
        // limit is the legacy 0x100 alias; its canonical value is verified by
        // the core Control Table tests.
        assert_eq!(REG_PRESENT_TEMP_MOSFET, REG_PRESENT_TEMPERATURE);
        assert_eq!(REG_TEMP_LIMIT_MOSFET, REG_TEMPERATURE_LIMIT);
        let mut buf2 = [0u8; 2];
        let mut buf4 = [0u8; 4];
        assert_eq!(map.get(REG_PRESENT_TEMP_MOSFET as u16, &mut buf2).unwrap(), 2);
        assert_eq!(u16::from_le_bytes(buf2), 55);

        assert_eq!(map.get(REG_PRESENT_TEMP_MCU as u16, &mut buf4).unwrap(), 4);
        assert_eq!(f32::from_le_bytes(buf4), 61.5);
        assert_eq!(map.get(REG_PRESENT_TEMP_MOTOR as u16, &mut buf4).unwrap(), 4);
        assert_eq!(f32::from_le_bytes(buf4), 72.25);
        assert_eq!(map.get(REG_TEMP_LIMIT_MCU as u16, &mut buf4).unwrap(), 4);
        assert_eq!(f32::from_le_bytes(buf4), 85.0);
        assert_eq!(map.get(REG_TEMP_LIMIT_MOTOR as u16, &mut buf4).unwrap(), 4);
        assert_eq!(f32::from_le_bytes(buf4), 90.0);

        // Limits are writable through the profile and read back unchanged.
        map.set(REG_TEMP_LIMIT_MCU as u16, &95.0f32.to_le_bytes()).unwrap();
        map.set(REG_TEMP_LIMIT_MOTOR as u16, &100.0f32.to_le_bytes()).unwrap();
        assert_eq!(map.get(REG_TEMP_LIMIT_MCU as u16, &mut buf4).unwrap(), 4);
        assert_eq!(f32::from_le_bytes(buf4), 95.0);
        assert_eq!(map.get(REG_TEMP_LIMIT_MOTOR as u16, &mut buf4).unwrap(), 4);
        assert_eq!(f32::from_le_bytes(buf4), 100.0);

        // Telemetry remains read-only from the bus.
        assert_eq!(
            map.write_phase(&[(REG_PRESENT_TEMP_MCU >> 8) as u8, REG_PRESENT_TEMP_MCU as u8, 1, 2, 3, 4]),
            Err(crate::table::Error::ReadOnly)
        );
        assert_eq!(map.get(REG_PRESENT_TEMP_MCU as u16, &mut buf4).unwrap(), 4);
        assert_eq!(f32::from_le_bytes(buf4), 61.5);
    }

    type ProfileServer = RegServer<
        ControlTableContext<MockStorage, MockBootloader>,
        CONTROL_TABLE_SIZE,
        128,
    >;

    /// Minimal host-side I2C loopback: master frames lower directly into the
    /// [`RegServer`] target. No heap and no hardware peripherals.
    struct LoopbackI2c<'a> {
        server: &'a mut ProfileServer,
    }

    impl eh_i2c::ErrorType for LoopbackI2c<'_> {
        type Error = eh_i2c::ErrorKind;
    }

    impl I2c for LoopbackI2c<'_> {
        fn write(&mut self, _address: u8, bytes: &[u8]) -> Result<(), Self::Error> {
            self.server.on_write(bytes).map_err(|_| {
                eh_i2c::ErrorKind::NoAcknowledge(eh_i2c::NoAcknowledgeSource::Address)
            })
        }

        fn read(&mut self, _address: u8, buf: &mut [u8]) -> Result<(), Self::Error> {
            self.server.on_read(buf.len(), buf).map(|_| ()).map_err(|_| {
                eh_i2c::ErrorKind::NoAcknowledge(eh_i2c::NoAcknowledgeSource::Address)
            })
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
    fn test_extended_registers_roundtrip_at_canonical_offsets_over_i2c() {
        const DEV: u8 = 0x30;
        let mut table = ControlTable::new();
        table.set_present_bus_power(123.5);
        table.set_hardware_capabilities(capabilities::DUAL_ENCODER | capabilities::POWER_TELEMETRY);
        table.set_watchdog_timeout_ms(250);
        table.set_total_revolutions(12345.5);
        table.set_cpu_load_pct(42);
        table.set_goal_torque_ff(0.0);

        let ctx = ControlTableContext::new(
            table,
            Some(MockStorage { save_count: 0 }),
            Some(MockBootloader { entered: false }),
        );
        let mut server: ProfileServer = RegServer::new(control_table_profile(ctx));

        // Read a representative f32 and the u32 capability mask at >= 0x100.
        {
            let mut bus = LoopbackI2c { server: &mut server };
            let mut client = RegClientI2c::<_, 16>::new(&mut bus);
            let mut out4 = [0u8; 4];
            client
                .read_register(DEV, REG_PRESENT_BUS_POWER as u16, 4, &mut out4)
                .unwrap();
            assert_eq!(f32::from_le_bytes(out4), 123.5);
            client
                .read_register(DEV, REG_HARDWARE_CAPABILITIES as u16, 4, &mut out4)
                .unwrap();
            assert_eq!(
                u32::from_le_bytes(out4),
                capabilities::DUAL_ENCODER | capabilities::POWER_TELEMETRY
            );
            // 64-bit odometer channel.
            let mut out8 = [0u8; 8];
            client
                .read_register(DEV, REG_TOTAL_REVOLUTIONS as u16, 8, &mut out8)
                .unwrap();
            assert_eq!(f64::from_le_bytes(out8), 12345.5);
        }
        assert_eq!(server.map().pointer(), (REG_TOTAL_REVOLUTIONS + 8) as u16);

        // Hooked write publishes the feedforward torque into the live table.
        {
            let mut bus = LoopbackI2c { server: &mut server };
            let mut client = RegClientI2c::<_, 16>::new(&mut bus);
            client
                .write_register(DEV, REG_GOAL_TORQUE_FF as u16, &0.75f32.to_le_bytes())
                .unwrap();
        }
        assert_eq!(server.map_mut().handler().table.goal_torque_ff(), 0.75);

        // Watchdog write hook re-arms the countdown.
        {
            let mut bus = LoopbackI2c { server: &mut server };
            let mut client = RegClientI2c::<_, 16>::new(&mut bus);
            client
                .write_register(DEV, REG_WATCHDOG_TIMEOUT_MS as u16, &250u16.to_le_bytes())
                .unwrap();
        }
        assert_eq!(server.map_mut().handler().table.watchdog_timeout_ms(), 250);
        assert_eq!(server.map_mut().handler().table.watchdog_time_left_ms(), 250);

        // RO capability register rejects bus writes (surface as NAK).
        {
            let mut bus = LoopbackI2c { server: &mut server };
            let mut client = RegClientI2c::<_, 16>::new(&mut bus);
            assert!(client
                .write_register(DEV, REG_HARDWARE_CAPABILITIES as u16, &0u32.to_le_bytes())
                .is_err());
        }
    }

    #[test]
    fn test_command_block_burst_16_bytes_roundtrip() {
        const DEV: u8 = 0x30;
        let table = ControlTable::new();
        let ctx = ControlTableContext::new(
            table,
            Some(MockStorage { save_count: 0 }),
            Some(MockBootloader { entered: false }),
        );
        let mut server: ProfileServer = RegServer::new(control_table_profile(ctx));

        // 16-byte contiguous command block at 0x12C..0x13B:
        // [0]     = REG_TORQUE_ENABLE (0x12C) = 1
        // [1]     = REG_OPERATING_MODE (0x12D) = 2 (Position)
        // [2..4]  = reserved padding (0x12E..0x12F) = [0, 0]
        // [4..8]  = REG_GOAL_POSITION (0x130) = 1.5f32 LE
        // [8..12] = REG_GOAL_VELOCITY (0x134) = 12.25f32 LE
        // [12..16]= REG_GOAL_CURRENT  (0x138) = 2.5f32 LE
        let mut payload = [0u8; 16];
        payload[0] = 1;
        payload[1] = 2;
        payload[4..8].copy_from_slice(&1.5f32.to_le_bytes());
        payload[8..12].copy_from_slice(&12.25f32.to_le_bytes());
        payload[12..16].copy_from_slice(&2.5f32.to_le_bytes());

        // Uses default RegClientI2c TX_BUF = 32 (no const-generic override needed).
        {
            let mut bus = LoopbackI2c { server: &mut server };
            let mut client: RegClientI2c<_> = RegClientI2c::new(&mut bus);
            client
                .write_register(DEV, REG_TORQUE_ENABLE as u16, &payload)
                .unwrap();

            let mut readback = [0u8; 16];
            client
                .read_register(DEV, REG_TORQUE_ENABLE as u16, 16, &mut readback)
                .unwrap();
            assert_eq!(readback, payload);
        }

        // Verify individual register reads from the server map match.
        let mut b1 = [0u8; 1];
        let mut b4 = [0u8; 4];
        server.map().get(REG_TORQUE_ENABLE as u16, &mut b1).unwrap();
        assert_eq!(b1[0], 1);
        server.map().get(REG_OPERATING_MODE as u16, &mut b1).unwrap();
        assert_eq!(b1[0], 2);
        server.map().get(REG_GOAL_POSITION as u16, &mut b4).unwrap();
        assert_eq!(f32::from_le_bytes(b4), 1.5);
        server.map().get(REG_GOAL_VELOCITY as u16, &mut b4).unwrap();
        assert_eq!(f32::from_le_bytes(b4), 12.25);
        server.map().get(REG_GOAL_CURRENT as u16, &mut b4).unwrap();
        assert_eq!(f32::from_le_bytes(b4), 2.5);
    }
}
