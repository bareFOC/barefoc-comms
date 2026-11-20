//! Fibre RPC schema and endpoints for Dynamixel-grade Smart Servo Control Table.

use barefoc::control_table::{
    ControlTable, OperatingMode, TopologyPreset, REG_ACCELERATION_LIMIT, REG_POS_P_GAIN,
    REG_VELOCITY_LIMIT, REG_VEL_P_GAIN,
};
use barefoc::traits::bootloader::BootloaderHandoff;
use barefoc::traits::storage::ConfigStorage;

use crate::schema::Endpoint0Handler;
use crate::server::Endpoint;

/// Minified JSON schema manifest for smart servo Fibre endpoints and RPC methods.
pub const BAREFOC_SMART_SERVO_JSON: &str = r#"[{"name":"servo","type":"object","members":[{"name":"assembly","type":"object","members":[{"name":"config","type":"object","members":[{"name":"topology_preset","id":1,"type":"uint32","access":"rw"}]}]},{"name":"axis0","type":"object","members":[{"name":"controller","type":"object","members":[{"name":"config","type":"object","members":[{"name":"control_mode","id":2,"type":"uint32","access":"rw"},{"name":"pos_gain","id":3,"type":"float","access":"rw"},{"name":"vel_gain","id":4,"type":"float","access":"rw"}]}]},{"name":"trap_traj","type":"object","members":[{"name":"config","type":"object","members":[{"name":"vel_limit","id":5,"type":"float","access":"rw"},{"name":"accel_limit","id":6,"type":"float","access":"rw"}]}]}]},{"name":"save_configuration","id":7,"type":"function","inputs":[],"outputs":[{"name":"status","type":"uint32"}]},{"name":"erase_configuration","id":8,"type":"function","inputs":[],"outputs":[{"name":"status","type":"uint32"}]},{"name":"reboot_bootloader","id":9,"type":"function","inputs":[],"outputs":[]}]}]"#;

/// Returns an `Endpoint0Handler` initialized with `BAREFOC_SMART_SERVO_JSON`.
pub fn create_smart_servo_schema() -> Endpoint0Handler {
    Endpoint0Handler::new(BAREFOC_SMART_SERVO_JSON)
}

/// Context passed to Fibre server holding the Control Table and lifecycle handlers.
pub struct SmartServoFibreContext<S = (), B = ()> {
    pub table: ControlTable,
    pub storage: Option<S>,
    pub bootloader: Option<B>,
    pub last_rpc_status: u32,
}

impl<S, B> SmartServoFibreContext<S, B> {
    pub fn new(table: ControlTable, storage: Option<S>, bootloader: Option<B>) -> Self {
        Self {
            table,
            storage,
            bootloader,
            last_rpc_status: 0,
        }
    }
}

/// Creates the list of 9 endpoints representing the smart servo schema.
pub fn create_smart_servo_endpoints<S: ConfigStorage, B: BootloaderHandoff>(
) -> [Endpoint<SmartServoFibreContext<S, B>>; 9] {
    [
        // ID 1: servo.assembly.config.topology_preset
        Endpoint::readwrite_u32(
            1,
            |ctx| ctx.table.topology_preset() as u32,
            |ctx, val| {
                if let Ok(preset) = TopologyPreset::try_from(val as u8) {
                    ctx.table.set_topology_preset(preset);
                }
            },
        ),
        // ID 2: servo.axis0.controller.config.control_mode
        Endpoint::readwrite_u32(
            2,
            |ctx| ctx.table.operating_mode() as u32,
            |ctx, val| {
                if let Ok(mode) = OperatingMode::try_from(val as u8) {
                    ctx.table.set_operating_mode(mode);
                }
            },
        ),
        // ID 3: servo.axis0.controller.config.pos_gain
        Endpoint::readwrite_f32(
            3,
            |ctx| ctx.table.read_f32(REG_POS_P_GAIN),
            |ctx, val| ctx.table.write_f32(REG_POS_P_GAIN, val),
        ),
        // ID 4: servo.axis0.controller.config.vel_gain
        Endpoint::readwrite_f32(
            4,
            |ctx| ctx.table.read_f32(REG_VEL_P_GAIN),
            |ctx, val| ctx.table.write_f32(REG_VEL_P_GAIN, val),
        ),
        // ID 5: servo.axis0.trap_traj.config.vel_limit
        Endpoint::readwrite_f32(
            5,
            |ctx| ctx.table.read_f32(REG_VELOCITY_LIMIT),
            |ctx, val| ctx.table.write_f32(REG_VELOCITY_LIMIT, val),
        ),
        // ID 6: servo.axis0.trap_traj.config.accel_limit
        Endpoint::readwrite_f32(
            6,
            |ctx| ctx.table.read_f32(REG_ACCELERATION_LIMIT),
            |ctx, val| ctx.table.write_f32(REG_ACCELERATION_LIMIT, val),
        ),
        // ID 7: servo.save_configuration()
        Endpoint::custom(
            7,
            Some(|ctx, out| {
                if out.len() >= 4 {
                    out[..4].copy_from_slice(&ctx.last_rpc_status.to_le_bytes());
                    4
                } else {
                    0
                }
            }),
            Some(|ctx, _| {
                if let Some(ref mut storage) = ctx.storage {
                    if storage.save_config(&ctx.table).is_ok() {
                        ctx.last_rpc_status = 0;
                        Ok(())
                    } else {
                        ctx.last_rpc_status = 1;
                        Err(())
                    }
                } else {
                    ctx.last_rpc_status = 2;
                    Err(())
                }
            }),
        ),
        // ID 8: servo.erase_configuration()
        Endpoint::custom(
            8,
            Some(|ctx, out| {
                if out.len() >= 4 {
                    out[..4].copy_from_slice(&ctx.last_rpc_status.to_le_bytes());
                    4
                } else {
                    0
                }
            }),
            Some(|ctx, _| {
                if let Some(ref mut storage) = ctx.storage {
                    if storage.factory_reset(&mut ctx.table).is_ok() {
                        ctx.last_rpc_status = 0;
                        Ok(())
                    } else {
                        ctx.last_rpc_status = 1;
                        Err(())
                    }
                } else {
                    ctx.table.load_factory_defaults();
                    ctx.last_rpc_status = 0;
                    Ok(())
                }
            }),
        ),
        // ID 9: servo.reboot_bootloader()
        Endpoint::custom(
            9,
            None,
            Some(|ctx, _| {
                if let Some(ref mut bootloader) = ctx.bootloader {
                    bootloader.enter_bootloader();
                } else {
                    Err(())
                }
            }),
        ),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::server::FibreServer;

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
            panic!("BOOTLOADER_REBOOT");
        }
    }

    #[test]
    fn test_smart_servo_schema_and_properties() {
        let table = ControlTable::new();
        let storage = MockStorage { save_count: 0 };
        let bootloader = MockBootloader { entered: false };
        let mut ctx = SmartServoFibreContext::new(table, Some(storage), Some(bootloader));

        let schema = create_smart_servo_schema();
        let endpoints = create_smart_servo_endpoints();
        let _server = FibreServer::new(schema, &endpoints);

        // Verify read/write property
        endpoints[1].write(&mut ctx, &3u32.to_le_bytes()).unwrap();
        assert_eq!(ctx.table.operating_mode(), OperatingMode::MultiTurnPosition);

        // Invoke save_configuration RPC
        endpoints[6].write(&mut ctx, &[]).unwrap();
        assert_eq!(ctx.storage.as_ref().unwrap().save_count, 1);
        assert_eq!(ctx.last_rpc_status, 0);

        let mut status_out = [0u8; 4];
        let len = endpoints[6].read(&ctx, &mut status_out);
        assert_eq!(len, 4);
        assert_eq!(u32::from_le_bytes(status_out), 0);
    }
}
