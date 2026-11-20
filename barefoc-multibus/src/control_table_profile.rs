//! # Universal Control Table Profile for MultiBus
//!
//! Exposes the standardized 176-byte [`ControlTable`] over MultiBus across byte
//! offsets `0x00..0xAF`, partitioning EEPROM registers (`0x00..0x5F`) as persistent
//! configuration and RAM registers (`0x60..0xAF`) as volatile operational targets and telemetry.
//!
//! Wires write hooks for:
//! - `SaveConfigCommand` (0x64): writes of `0x55` invoke `ConfigStorage::save_config`
//!   and self-clear.
//! - `RebootCommand` (0x63): writes of `0xF0` invoke `BootloaderHandoff::enter_bootloader`.

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
}

impl<S, B> ControlTableContext<S, B> {
    /// Create a new context with a Control Table and optional drivers.
    pub fn new(table: ControlTable, storage: Option<S>, bootloader: Option<B>) -> Self {
        Self {
            table,
            storage,
            bootloader,
        }
    }
}

fn save_hook<S: ConfigStorage, B>(
    ctx: &mut ControlTableContext<S, B>,
    _offset: u8,
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
    _offset: u8,
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

/// Build a MultiBus [`RegMap`] from a [`ControlTableContext`].
pub fn control_table_profile<S: ConfigStorage, B: BootloaderHandoff>(
    ctx: ControlTableContext<S, B>,
) -> RegMap<ControlTableContext<S, B>, 176, 64> {
    let mut map = RegMap::new(ctx);

    // -----------------------------------------------------------------------
    // EEPROM Area (0x00..0x5F)
    // -----------------------------------------------------------------------
    let _ = map.add_register(RegEntry::ro(REG_MODEL_NUMBER as u8, 2));
    let _ = map.add_register(RegEntry::ro(REG_FIRMWARE_VERSION as u8, 1));
    let _ = map.add_register(RegEntry::ro(REG_PROTOCOL_VERSION as u8, 1));
    let _ = map.add_register(RegEntry::rw(REG_NODE_ID as u8, 1));
    let _ = map.add_register(RegEntry::rw(REG_OPERATING_MODE as u8, 1));
    let _ = map.add_register(RegEntry::rw(REG_SECONDARY_ID as u8, 1));
    let _ = map.add_register(RegEntry::rw(REG_TOPOLOGY_PRESET as u8, 1));
    let _ = map.add_register(RegEntry::rw(REG_COUPLING_MASK as u8, 1));

    let _ = map.add_register(RegEntry::rw(REG_HOMING_OFFSET as u8, 4));
    let _ = map.add_register(RegEntry::rw(REG_CURRENT_LIMIT as u8, 4));
    let _ = map.add_register(RegEntry::rw(REG_ACCELERATION_LIMIT as u8, 4));
    let _ = map.add_register(RegEntry::rw(REG_VELOCITY_LIMIT as u8, 4));
    let _ = map.add_register(RegEntry::rw(REG_MIN_POSITION_LIMIT as u8, 4));
    let _ = map.add_register(RegEntry::rw(REG_MAX_POSITION_LIMIT as u8, 4));
    let _ = map.add_register(RegEntry::rw(REG_VOLTAGE_LIMIT as u8, 4));
    let _ = map.add_register(RegEntry::rw(REG_TEMPERATURE_LIMIT as u8, 4));
    let _ = map.add_register(RegEntry::rw(REG_MIN_BUS_VOLTAGE as u8, 4));
    let _ = map.add_register(RegEntry::rw(REG_MAX_BUS_VOLTAGE as u8, 4));

    let _ = map.add_register(RegEntry::rw(REG_POS_P_GAIN as u8, 4));
    let _ = map.add_register(RegEntry::rw(REG_POS_I_GAIN as u8, 4));
    let _ = map.add_register(RegEntry::rw(REG_POS_D_GAIN as u8, 4));
    let _ = map.add_register(RegEntry::rw(REG_VEL_P_GAIN as u8, 4));
    let _ = map.add_register(RegEntry::rw(REG_VEL_I_GAIN as u8, 4));
    let _ = map.add_register(RegEntry::rw(REG_VEL_D_GAIN as u8, 4));
    let _ = map.add_register(RegEntry::rw(REG_CURRENT_P_GAIN as u8, 4));
    let _ = map.add_register(RegEntry::rw(REG_CURRENT_I_GAIN as u8, 4));
    let _ = map.add_register(RegEntry::rw(REG_CURRENT_D_GAIN as u8, 4));

    // -----------------------------------------------------------------------
    // RAM Area (0x60..0xAF)
    // -----------------------------------------------------------------------
    let _ = map.add_register(RegEntry::rw(REG_TORQUE_ENABLE as u8, 1));
    let _ = map.add_register(RegEntry::rw(REG_LED as u8, 1));
    let _ = map.add_register(RegEntry::rw(REG_STATUS_RETURN_LEVEL as u8, 1));

    // Reboot hook (0x63)
    let _ = map.add_register(RegEntry::rw_hooked(
        REG_REBOOT_CMD as u8,
        1,
        reboot_hook::<S, B>,
    ));

    // Save config hook (0x64)
    let _ = map.add_register(RegEntry::rw_hooked(
        REG_SAVE_CONFIG_CMD as u8,
        1,
        save_hook::<S, B>,
    ));

    let _ = map.add_register(RegEntry::rw(REG_PROFILE_VELOCITY as u8, 4));
    let _ = map.add_register(RegEntry::rw(REG_PROFILE_ACCELERATION as u8, 4));
    let _ = map.add_register(RegEntry::rw(REG_GOAL_POSITION as u8, 4));
    let _ = map.add_register(RegEntry::rw(REG_GOAL_VELOCITY as u8, 4));
    let _ = map.add_register(RegEntry::rw(REG_GOAL_CURRENT as u8, 4));
    let _ = map.add_register(RegEntry::rw(REG_GOAL_VOLTAGE as u8, 4));

    // Telemetry burst read block (0x80..0xA0, 32 bytes)
    let _ = map.add_register(RegEntry::ro(REG_HARDWARE_ERROR_STATUS as u8, 2));
    let _ = map.add_register(RegEntry::ro(REG_PRESENT_TEMPERATURE as u8, 2));
    let _ = map.add_register(RegEntry::ro(REG_PRESENT_POSITION as u8, 4));
    let _ = map.add_register(RegEntry::ro(REG_PRESENT_VELOCITY as u8, 4));
    let _ = map.add_register(RegEntry::ro(REG_PRESENT_CURRENT as u8, 4));
    let _ = map.add_register(RegEntry::ro(REG_PRESENT_VOLTAGE as u8, 4));
    let _ = map.add_register(RegEntry::ro(REG_PRESENT_BUS_VOLTAGE as u8, 4));
    let _ = map.add_register(RegEntry::ro(REG_FILTERED_CURRENT as u8, 4));
    let _ = map.add_register(RegEntry::ro(REG_TRAJECTORY_POSITION as u8, 4));

    let _ = map.add_register(RegEntry::ro(REG_MOVING as u8, 1));
    let _ = map.add_register(RegEntry::ro(REG_MULTI_TURN_REVOLUTIONS as u8, 4));
    let _ = map.add_register(RegEntry::ro(REG_MULTI_TURN_ANGLE as u8, 4));

    // Seed map bank from initial context table
    sync_all_to_map(&mut map);

    map
}

/// Synchronize all registers from `handler().table` into the MultiBus bank.
pub fn sync_all_to_map<S, B>(map: &mut RegMap<ControlTableContext<S, B>, 176, 64>) {
    let t = map.handler().table;
    let _ = map.set(REG_MODEL_NUMBER as u8, &t.read_u16(REG_MODEL_NUMBER).to_le_bytes());
    let _ = map.set(REG_FIRMWARE_VERSION as u8, &[t.read_u8(REG_FIRMWARE_VERSION)]);
    let _ = map.set(REG_PROTOCOL_VERSION as u8, &[t.read_u8(REG_PROTOCOL_VERSION)]);
    let _ = map.set(REG_NODE_ID as u8, &[t.node_id()]);
    let _ = map.set(REG_OPERATING_MODE as u8, &[t.read_u8(REG_OPERATING_MODE)]);
    let _ = map.set(REG_SECONDARY_ID as u8, &[t.read_u8(REG_SECONDARY_ID)]);
    let _ = map.set(REG_TOPOLOGY_PRESET as u8, &[t.read_u8(REG_TOPOLOGY_PRESET)]);
    let _ = map.set(REG_COUPLING_MASK as u8, &[t.coupling_mask()]);

    let _ = map.set(REG_HOMING_OFFSET as u8, &t.read_f32(REG_HOMING_OFFSET).to_le_bytes());
    let _ = map.set(REG_CURRENT_LIMIT as u8, &t.current_limit().to_le_bytes());
    let _ = map.set(REG_ACCELERATION_LIMIT as u8, &t.acceleration_limit().to_le_bytes());
    let _ = map.set(REG_VELOCITY_LIMIT as u8, &t.velocity_limit().to_le_bytes());
    let _ = map.set(REG_MIN_POSITION_LIMIT as u8, &t.min_position_limit().to_le_bytes());
    let _ = map.set(REG_MAX_POSITION_LIMIT as u8, &t.max_position_limit().to_le_bytes());
    let _ = map.set(REG_VOLTAGE_LIMIT as u8, &t.voltage_limit().to_le_bytes());
    let _ = map.set(REG_TEMPERATURE_LIMIT as u8, &t.temperature_limit().to_le_bytes());
    let _ = map.set(REG_MIN_BUS_VOLTAGE as u8, &t.min_bus_voltage().to_le_bytes());
    let _ = map.set(REG_MAX_BUS_VOLTAGE as u8, &t.max_bus_voltage().to_le_bytes());

    let _ = map.set(REG_POS_P_GAIN as u8, &t.read_f32(REG_POS_P_GAIN).to_le_bytes());
    let _ = map.set(REG_POS_I_GAIN as u8, &t.read_f32(REG_POS_I_GAIN).to_le_bytes());
    let _ = map.set(REG_POS_D_GAIN as u8, &t.read_f32(REG_POS_D_GAIN).to_le_bytes());
    let _ = map.set(REG_VEL_P_GAIN as u8, &t.read_f32(REG_VEL_P_GAIN).to_le_bytes());
    let _ = map.set(REG_VEL_I_GAIN as u8, &t.read_f32(REG_VEL_I_GAIN).to_le_bytes());
    let _ = map.set(REG_VEL_D_GAIN as u8, &t.read_f32(REG_VEL_D_GAIN).to_le_bytes());
    let _ = map.set(REG_CURRENT_P_GAIN as u8, &t.read_f32(REG_CURRENT_P_GAIN).to_le_bytes());
    let _ = map.set(REG_CURRENT_I_GAIN as u8, &t.read_f32(REG_CURRENT_I_GAIN).to_le_bytes());
    let _ = map.set(REG_CURRENT_D_GAIN as u8, &t.read_f32(REG_CURRENT_D_GAIN).to_le_bytes());

    let _ = map.set(REG_TORQUE_ENABLE as u8, &[if t.torque_enable() { 1 } else { 0 }]);
    let _ = map.set(REG_LED as u8, &[t.read_u8(REG_LED)]);
    let _ = map.set(REG_STATUS_RETURN_LEVEL as u8, &[t.read_u8(REG_STATUS_RETURN_LEVEL)]);
    let _ = map.set(REG_PROFILE_VELOCITY as u8, &t.read_f32(REG_PROFILE_VELOCITY).to_le_bytes());
    let _ = map.set(REG_PROFILE_ACCELERATION as u8, &t.read_f32(REG_PROFILE_ACCELERATION).to_le_bytes());
    let _ = map.set(REG_GOAL_POSITION as u8, &t.goal_position().to_le_bytes());
    let _ = map.set(REG_GOAL_VELOCITY as u8, &t.goal_velocity().to_le_bytes());
    let _ = map.set(REG_GOAL_CURRENT as u8, &t.goal_current().to_le_bytes());
    let _ = map.set(REG_GOAL_VOLTAGE as u8, &t.goal_voltage().to_le_bytes());

    let _ = map.set(REG_HARDWARE_ERROR_STATUS as u8, &t.hardware_error_status().to_le_bytes());
    let _ = map.set(REG_PRESENT_TEMPERATURE as u8, &t.read_u16(REG_PRESENT_TEMPERATURE).to_le_bytes());
    let _ = map.set(REG_PRESENT_POSITION as u8, &t.read_f32(REG_PRESENT_POSITION).to_le_bytes());
    let _ = map.set(REG_PRESENT_VELOCITY as u8, &t.read_f32(REG_PRESENT_VELOCITY).to_le_bytes());
    let _ = map.set(REG_PRESENT_CURRENT as u8, &t.read_f32(REG_PRESENT_CURRENT).to_le_bytes());
    let _ = map.set(REG_PRESENT_VOLTAGE as u8, &t.read_f32(REG_PRESENT_VOLTAGE).to_le_bytes());
    let _ = map.set(REG_PRESENT_BUS_VOLTAGE as u8, &t.read_f32(REG_PRESENT_BUS_VOLTAGE).to_le_bytes());
    let _ = map.set(REG_FILTERED_CURRENT as u8, &t.read_f32(REG_FILTERED_CURRENT).to_le_bytes());
    let _ = map.set(REG_TRAJECTORY_POSITION as u8, &t.read_f32(REG_TRAJECTORY_POSITION).to_le_bytes());
    let _ = map.set(REG_MOVING as u8, &[t.read_u8(REG_MOVING)]);
    let _ = map.set(REG_MULTI_TURN_REVOLUTIONS as u8, &t.read_i32(REG_MULTI_TURN_REVOLUTIONS).to_le_bytes());
    let _ = map.set(REG_MULTI_TURN_ANGLE as u8, &t.read_f32(REG_MULTI_TURN_ANGLE).to_le_bytes());
}

#[cfg(test)]
mod tests {
    use super::*;

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

        // Latch pointer to 0x80
        map.write_phase(&[0x80]).unwrap();

        // Read burst of 32 bytes
        let mut burst_buf = [0u8; 32];
        let n = map.read_phase(32, &mut burst_buf).unwrap();
        assert_eq!(n, 32);

        // Hardware error status at 0x80 (offset 0 in burst)
        let err_status = u16::from_le_bytes([burst_buf[0], burst_buf[1]]);
        assert_eq!(err_status, 0x0001);

        // Present position at 0x84 (offset 4 in burst)
        let pos = f32::from_le_bytes([burst_buf[4], burst_buf[5], burst_buf[6], burst_buf[7]]);
        assert!((pos - 3.14159).abs() < 1e-4);

        // Present bus voltage at 0x94 (offset 20 in burst)
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

        // Write 0x55 to SaveConfigCommand (0x64)
        map.write_phase(&[0x64, 0x55]).unwrap();

        // Verify save_config was called
        assert_eq!(map.handler().storage.as_ref().unwrap().save_count, 1);

        // Register must self-clear
        let mut val = [0u8; 1];
        let _ = map.get(0x64, &mut val).unwrap();
        assert_eq!(val[0], 0x00);
    }
}
