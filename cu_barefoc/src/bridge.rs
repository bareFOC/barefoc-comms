//! `CuBridge` implementation for BareFOC actuator networks.
//!
//! Lifecycle mapping (see `design.md`, decision 4):
//!
//! - `new` — discovers nodes with [`BusScanner`] and caches unit conversions.
//! - `start` — arms torque only when Tx writers are attached.
//! - `preprocess` — polls position, velocity, current and temperature.
//! - `send` — emits exactly one `INST_SYNC_WRITE` frame.
//! - `receive` — publishes the cached telemetry snapshot.
//! - `stop` — broadcasts `REG_TORQUE_ENABLE = 0`.
//!
//! Upstream: `ref/copper-rs/core/cu29_runtime/src/cubridge.rs` (`CuBridge`),
//! `barefoc-master/src/{scanner,client,sync}.rs`.

use barefoc_master::registers::{CAP_CURRENT_SENSE, CAP_TEMPERATURE, REG_HARDWARE_CAPABILITIES};
use barefoc_master::sync::{build_sync_write_frame, MAX_PACKET_LEN};
use barefoc_master::{
    BusScanner, ControlTableClient, NodeDescriptor, SyncGroup, REG_GOAL_CURRENT, REG_GOAL_POSITION,
    REG_GOAL_VELOCITY, REG_PRESENT_POSITION, REG_PRESENT_VELOCITY, REG_TORQUE_ENABLE,
};
use cu29::config::ComponentConfig;
use cu29::context::CuContext;
use cu29::cubridge::{BridgeChannel, BridgeChannelConfig, BridgeChannelSet, CuBridge};
use cu29::cutask::{CuMsg, CuMsgPayload, Freezable};
use cu29::reflect::Reflect;
use cu29::rx_channels;
use cu29::tx_channels;
use cu29_traits::CuResult;

use crate::messages::{
    JointPositions, JointTelemetry, JointTorques, JointVelocities, MAX_ACTUATORS,
};
use crate::transport::{ActuatorBus, MasterTransport};
use crate::{BridgeError, REG_PRESENT_TEMPERATURE};

// Outgoing joint-setpoint channels (Copper task graph -> actuators).
// Upstream: `ref/copper-rs/core/cu29_runtime/src/cubridge.rs` (`tx_channels!`).
tx_channels! {
    pub struct BarefocTxChannels : BarefocTxId {
        /// Desired joint angles.
        joint_positions => JointPositions,
        /// Desired joint velocities.
        joint_velocities => JointVelocities,
        /// Desired torque-producing currents.
        joint_torques => JointTorques,
    }
}

// Incoming telemetry channel (actuators -> Copper task graph).
// Upstream: `ref/copper-rs/core/cu29_runtime/src/cubridge.rs` (`rx_channels!`).
rx_channels! {
    pub struct BarefocRxChannels : BarefocRxId {
        /// Cached present position/velocity/current/temperature snapshot.
        joint_telemetry => JointTelemetry,
    }
}

/// Angular unit used on all joint channels.
///
/// BareFOC control-table registers carry radians; conversion happens at the
/// bridge boundary so planners can work in their natural unit.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum UnitMode {
    /// Radians (device-native, default).
    #[default]
    Rad,
    /// Raw register values (no conversion).
    Raw,
    /// Degrees.
    Deg,
    /// Normalized `[-1, 1]` over `[-pi, pi]`.
    Normalize,
}

impl UnitMode {
    /// Parse the `units` configuration key.
    pub fn parse(value: &str) -> CuResult<Self> {
        match value {
            "rad" => Ok(Self::Rad),
            "raw" => Ok(Self::Raw),
            "deg" => Ok(Self::Deg),
            "normalize" => Ok(Self::Normalize),
            _ => Err(BridgeError::Config("units").into()),
        }
    }

    /// Convert a device-native value (radians) into the configured unit.
    pub fn from_device(self, value: f32) -> f32 {
        match self {
            Self::Raw | Self::Rad => value,
            Self::Deg => value * (180.0 / core::f32::consts::PI),
            Self::Normalize => value / core::f32::consts::PI,
        }
    }

    /// Convert a value in the configured unit into a device-native value.
    pub fn to_device(self, value: f32) -> f32 {
        match self {
            Self::Raw | Self::Rad => value,
            Self::Deg => value * (core::f32::consts::PI / 180.0),
            Self::Normalize => value * core::f32::consts::PI,
        }
    }
}

/// Copper bridge for a multidrop BareFOC actuator network.
///
/// Generic over the [`ActuatorBus`] so host tests can use [`MockBus`] and
/// deployments can use `SerialBus`, `I2cBus` or `CanBus` without changing
/// lifecycle logic.
///
/// Upstream: `ref/copper-rs/components/bridges/cu_feetech/src/lib.rs`
/// (`FeetechBridge` lifecycle and channel pattern).
#[derive(Reflect)]
#[reflect(from_reflect = false)]
pub struct BarefocBridge<B: ActuatorBus> {
    /// Owned transport adapter.
    #[reflect(ignore)]
    bus: MasterTransport<B>,
    /// Discovered node inventory, cached at construction.
    nodes: heapless::Vec<NodeDescriptor, MAX_ACTUATORS>,
    /// Unit conversion mode.
    #[reflect(ignore)]
    units: UnitMode,
    /// Per-node capability bitmask, indexed like `nodes`.
    capabilities: [u32; MAX_ACTUATORS],
    /// True when at least one Tx channel writer is attached.
    has_writers: bool,
    /// True between `start` and `stop`.
    running: bool,
    /// True after torque has been armed.
    armed: bool,
    /// Latest telemetry snapshot.
    telemetry: JointTelemetry,
}

impl<B: ActuatorBus> BarefocBridge<B> {
    /// Number of nodes discovered during construction.
    pub fn node_count(&self) -> usize {
        self.nodes.len()
    }

    /// Borrow the cached node inventory.
    pub fn nodes(&self) -> &[NodeDescriptor] {
        &self.nodes
    }

    /// Configured unit mode.
    pub fn units(&self) -> UnitMode {
        self.units
    }

    /// Whether torque is currently armed.
    pub fn is_armed(&self) -> bool {
        self.armed
    }

    /// Whether the bridge is between `start` and `stop`.
    pub fn is_running(&self) -> bool {
        self.running
    }

    /// Borrow the cached telemetry snapshot.
    pub fn telemetry(&self) -> &JointTelemetry {
        &self.telemetry
    }

    /// Re-scan the bus and refresh the cached inventory.
    ///
    /// Upstream: `barefoc-master/src/scanner.rs` (`BusScanner::scan_i2c`).
    pub fn rescan(&mut self, start: u8, end: u8) -> CuResult<usize> {
        let found = {
            let mut scanner = BusScanner::new(&mut self.bus);
            scanner.scan_i2c(start..=end).map_err(BridgeError::from)?
        };
        if found.is_empty() {
            return Err(BridgeError::NoNodes.into());
        }
        let mut nodes = heapless::Vec::new();
        for descriptor in found.iter().take(MAX_ACTUATORS) {
            nodes.push(*descriptor).map_err(|_| BridgeError::Capacity {
                requested: found.len(),
                maximum: MAX_ACTUATORS,
            })?;
        }
        let count = nodes.len();
        self.nodes = nodes;
        Ok(count)
    }
}

// `Freezable` uses its stateless default implementation: bridge state is
// re-derived from the bus on every `new`/`preprocess` cycle, and deterministic
// record/replay uses the reflectable messages. See design decision 5.
impl<B: ActuatorBus> Freezable for BarefocBridge<B> {}

impl<B: ActuatorBus + 'static> CuBridge for BarefocBridge<B> {
    type Tx = BarefocTxChannels;
    type Rx = BarefocRxChannels;
    type Resources<'r> = B;

    fn new(
        config: Option<&ComponentConfig>,
        tx_channels: &[BridgeChannelConfig<<Self::Tx as BridgeChannelSet>::Id>],
        _rx_channels: &[BridgeChannelConfig<<Self::Rx as BridgeChannelSet>::Id>],
        resources: Self::Resources<'_>,
    ) -> CuResult<Self>
    where
        Self: Sized,
    {
        let units = match config {
            Some(cfg) => match cfg.get::<String>("units")? {
                Some(raw) => UnitMode::parse(&raw)?,
                None => UnitMode::default(),
            },
            None => UnitMode::default(),
        };
        let scan_start = match config {
            Some(cfg) => cfg.get::<u8>("scan_start")?.unwrap_or(0x08),
            None => 0x08,
        };
        let scan_end = match config {
            Some(cfg) => cfg.get::<u8>("scan_end")?.unwrap_or(0x77),
            None => 0x77,
        };

        let mut bus = MasterTransport::new(resources);
        let found = {
            let mut scanner = BusScanner::new(&mut bus);
            scanner
                .scan_i2c(scan_start..=scan_end)
                .map_err(BridgeError::from)?
        };
        if found.is_empty() {
            return Err(BridgeError::NoNodes.into());
        }

        let mut nodes = heapless::Vec::new();
        let mut capabilities = [0u32; MAX_ACTUATORS];
        for (index, descriptor) in found.iter().enumerate().take(MAX_ACTUATORS) {
            nodes.push(*descriptor).map_err(|_| BridgeError::Capacity {
                requested: found.len(),
                maximum: MAX_ACTUATORS,
            })?;
            // A node without the extended register table is treated as having
            // no optional telemetry (all such channels publish NaN).
            let mut client = ControlTableClient::new(descriptor.address, &mut bus);
            capabilities[index] = client.read_u32(REG_HARDWARE_CAPABILITIES).unwrap_or(0);
        }

        Ok(Self {
            bus,
            nodes,
            units,
            capabilities,
            has_writers: !tx_channels.is_empty(),
            running: false,
            armed: false,
            telemetry: JointTelemetry::default(),
        })
    }

    fn start(&mut self, _ctx: &CuContext) -> CuResult<()> {
        // Teach/follower mode: no Tx writers means torque must stay off.
        if self.has_writers && !self.armed {
            for node in self.nodes.clone() {
                let mut client = ControlTableClient::new(node.address, &mut self.bus);
                client
                    .write_u8(REG_TORQUE_ENABLE, 1)
                    .map_err(BridgeError::from)?;
            }
            self.armed = true;
        }
        self.running = true;
        Ok(())
    }

    fn preprocess(&mut self, _ctx: &CuContext) -> CuResult<()> {
        let units = self.units;
        let nodes = self.nodes.clone();
        let capabilities = self.capabilities;
        let mut positions = [f32::NAN; MAX_ACTUATORS];
        let mut velocities = [f32::NAN; MAX_ACTUATORS];
        let mut currents = [f32::NAN; MAX_ACTUATORS];
        let mut temperatures = [f32::NAN; MAX_ACTUATORS];

        for (index, node) in nodes.iter().enumerate() {
            let mut client = ControlTableClient::new(node.address, &mut self.bus);
            positions[index] = units.from_device(
                client
                    .read_f32(REG_PRESENT_POSITION)
                    .map_err(BridgeError::from)?,
            );
            velocities[index] = units.from_device(
                client
                    .read_f32(REG_PRESENT_VELOCITY)
                    .map_err(BridgeError::from)?,
            );
            if capabilities[index] & CAP_CURRENT_SENSE != 0 {
                currents[index] = client
                    .read_f32(barefoc_master::REG_PRESENT_CURRENT)
                    .map_err(BridgeError::from)?;
            }
            if capabilities[index] & CAP_TEMPERATURE != 0 {
                let raw = client
                    .read_u16(REG_PRESENT_TEMPERATURE)
                    .map_err(BridgeError::from)?;
                if raw != u16::MAX {
                    temperatures[index] = raw as f32;
                }
            }
        }

        self.telemetry
            .set_from_arrays(nodes.len(), positions, velocities, currents, temperatures);
        Ok(())
    }

    fn send<'a, Payload>(
        &mut self,
        _ctx: &CuContext,
        channel: &'static BridgeChannel<<Self::Tx as BridgeChannelSet>::Id, Payload>,
        msg: &CuMsg<Payload>,
    ) -> CuResult<()>
    where
        Payload: CuMsgPayload + 'a,
    {
        let units = self.units;
        let (register, values) = match channel.id {
            BarefocTxId::JointPositions => {
                let payload = payload_of::<JointPositions>(msg, "joint_positions")?;
                (
                    REG_GOAL_POSITION,
                    to_device_values(payload.as_slice(), units)?,
                )
            }
            BarefocTxId::JointVelocities => {
                let payload = payload_of::<JointVelocities>(msg, "joint_velocities")?;
                (
                    REG_GOAL_VELOCITY,
                    to_device_values(payload.as_slice(), units)?,
                )
            }
            BarefocTxId::JointTorques => {
                let payload = payload_of::<JointTorques>(msg, "joint_torques")?;
                (
                    REG_GOAL_CURRENT,
                    to_device_values(payload.as_slice(), units)?,
                )
            }
        };

        let targets = build_targets(&values, &self.nodes)?;
        if register == REG_GOAL_POSITION {
            // Reuse the SDK coordinator: one atomic INST_SYNC_WRITE frame.
            let mut group = SyncGroup::new(&mut self.bus);
            group
                .sync_write_position(&targets)
                .map_err(BridgeError::from)?;
        } else {
            // Velocity/current use the same SDK frame builder with their own
            // control-table address; still exactly one emitted frame.
            let mut frame: heapless::Vec<u8, MAX_PACKET_LEN> = heapless::Vec::new();
            frame
                .resize(MAX_PACKET_LEN, 0)
                .map_err(|_| BridgeError::Bus(crate::BusError::BufferTooSmall))?;
            let len = build_sync_write_frame(register, &targets, &mut frame)
                .map_err(BridgeError::from)?;
            frame.truncate(len);
            self.bus
                .send_sync_frame(&frame)
                .map_err(BridgeError::from)?;
        }
        Ok(())
    }

    fn receive<'a, Payload>(
        &mut self,
        _ctx: &CuContext,
        channel: &'static BridgeChannel<<Self::Rx as BridgeChannelSet>::Id, Payload>,
        msg: &mut CuMsg<Payload>,
    ) -> CuResult<()>
    where
        Payload: CuMsgPayload + 'a,
    {
        match channel.id {
            BarefocRxId::JointTelemetry => {
                let typed = msg.downcast_mut::<JointTelemetry>()?;
                typed.set_payload(self.telemetry.clone());
                Ok(())
            }
        }
    }

    fn stop(&mut self, _ctx: &CuContext) -> CuResult<()> {
        {
            let mut group = SyncGroup::new(&mut self.bus);
            group.emergency_stop().map_err(BridgeError::from)?;
        }
        self.armed = false;
        self.running = false;
        Ok(())
    }
}

/// Borrow a payload from a Copper message with a typed empty-payload error.
fn payload_of<'m, P: CuMsgPayload + 'static>(
    msg: &'m CuMsg<impl CuMsgPayload + 'static>,
    name: &'static str,
) -> CuResult<&'m P> {
    let typed = msg.downcast_ref::<P>()?;
    typed
        .payload()
        .ok_or_else(|| BridgeError::MissingPayload(name).into())
}

/// Convert joint values into device units with a fixed-capacity buffer.
fn to_device_values(
    values: &[f32],
    units: UnitMode,
) -> CuResult<heapless::Vec<f32, MAX_ACTUATORS>> {
    if values.len() > MAX_ACTUATORS {
        return Err(BridgeError::Capacity {
            requested: values.len(),
            maximum: MAX_ACTUATORS,
        }
        .into());
    }
    let mut converted = heapless::Vec::new();
    for &value in values {
        converted
            .push(units.to_device(value))
            .map_err(|_| BridgeError::Capacity {
                requested: values.len(),
                maximum: MAX_ACTUATORS,
            })?;
    }
    Ok(converted)
}

/// Pair each value with its discovered node address.
fn build_targets(
    values: &[f32],
    nodes: &[NodeDescriptor],
) -> CuResult<heapless::Vec<(u8, f32), MAX_ACTUATORS>> {
    if values.len() > nodes.len() {
        return Err(BridgeError::Capacity {
            requested: values.len(),
            maximum: nodes.len(),
        }
        .into());
    }
    let mut targets = heapless::Vec::new();
    for (value, node) in values.iter().zip(nodes.iter()) {
        targets
            .push((node.address, *value))
            .map_err(|_| BridgeError::Capacity {
                requested: values.len(),
                maximum: MAX_ACTUATORS,
            })?;
    }
    Ok(targets)
}

#[cfg(all(test, feature = "mock"))]
mod tests {
    use super::*;
    use crate::transport::MockBus;
    use crate::REG_PRESENT_TEMPERATURE;
    use barefoc_master::{BROADCAST_ID, REG_PRESENT_CURRENT};
    use cu29::bincode::{Decode, Encode};

    fn bus_with_nodes(nodes: &[u8]) -> MockBus {
        let mut bus = MockBus::new();
        for &node in nodes {
            bus.add_node(node, 0x0503, 7, [node; 12]);
            bus.set_register(node, REG_PRESENT_POSITION, &1.0f32.to_le_bytes())
                .unwrap();
            bus.set_register(node, REG_PRESENT_VELOCITY, &2.0f32.to_le_bytes())
                .unwrap();
            bus.set_register(node, REG_PRESENT_CURRENT, &0.5f32.to_le_bytes())
                .unwrap();
            bus.set_register(node, REG_PRESENT_TEMPERATURE, &25u16.to_le_bytes())
                .unwrap();
        }
        bus
    }

    fn config(units: Option<&str>) -> ComponentConfig {
        let mut cfg = ComponentConfig::default();
        if let Some(units) = units {
            cfg.set("units", units.to_string());
        }
        cfg
    }

    fn tx_descriptors() -> [BridgeChannelConfig<BarefocTxId>; 1] {
        [BridgeChannelConfig::from_static(
            &BarefocTxChannels::JOINT_POSITIONS,
            None,
            None,
        )]
    }

    fn make_bridge(bus: MockBus, with_tx: bool, units: Option<&str>) -> BarefocBridge<MockBus> {
        let cfg = config(units);
        let tx: &[BridgeChannelConfig<BarefocTxId>] = if with_tx { &tx_descriptors() } else { &[] };
        BarefocBridge::new(Some(&cfg), tx, &[], bus).unwrap()
    }

    fn context() -> CuContext {
        CuContext::new_mock_clock().0
    }

    #[test]
    fn new_discovers_nodes_and_caches_units() {
        let bridge = make_bridge(bus_with_nodes(&[0x40, 0x41]), true, Some("deg"));
        assert_eq!(bridge.node_count(), 2);
        assert_eq!(bridge.nodes()[0].address, 0x40);
        assert_eq!(bridge.units(), UnitMode::Deg);
        assert!(bridge.has_writers);
        assert!(!bridge.is_armed());
    }

    #[test]
    fn new_reports_typed_error_when_no_nodes_answer() {
        let err = BarefocBridge::new(Some(&config(None)), &[], &[], MockBus::new())
            .err()
            .expect("empty bus must be rejected");
        assert!(err.to_string().contains("BareFOC"));
    }

    #[test]
    fn start_arms_only_with_tx_writers() {
        ctx_test(|| {
            let ctx = context();
            let mut commanded = make_bridge(bus_with_nodes(&[0x40]), true, None);
            commanded.start(&ctx).unwrap();
            assert!(commanded.is_armed());
            assert!(commanded.is_running());
            assert!(commanded.bus.inner().writes.iter().any(|write| {
                write.node == 0x40 && write.reg == REG_TORQUE_ENABLE && &write.bytes[..] == &[1]
            }));

            let mut teaching = make_bridge(bus_with_nodes(&[0x40]), false, None);
            teaching.start(&ctx).unwrap();
            assert!(!teaching.is_armed());
            assert!(teaching.is_running());
            assert!(teaching.bus.inner().writes.is_empty());
        });
    }

    #[test]
    fn preprocess_refreshes_all_telemetry_with_nan_for_unequipped() {
        ctx_test(|| {
            let ctx = context();
            let mut bus = bus_with_nodes(&[0x40, 0x41]);
            // Second node has no current sense nor temperature channel.
            bus.set_register(
                0x41,
                barefoc_master::registers::REG_HARDWARE_CAPABILITIES,
                &0u32.to_le_bytes(),
            )
            .unwrap();
            let mut bridge = make_bridge(bus, false, None);
            bridge.preprocess(&ctx).unwrap();
            assert_eq!(bridge.telemetry().positions.as_slice(), &[1.0, 1.0]);
            assert_eq!(bridge.telemetry().velocities.as_slice(), &[2.0, 2.0]);
            assert_eq!(bridge.telemetry().currents.as_slice()[0], 0.5);
            assert!(bridge.telemetry().currents.as_slice()[1].is_nan());
            assert_eq!(bridge.telemetry().temperatures.as_slice()[0], 25.0);
            assert!(bridge.telemetry().temperatures.as_slice()[1].is_nan());
        });
    }

    #[test]
    fn send_emits_one_exact_sync_frame_for_multiple_nodes() {
        ctx_test(|| {
            let ctx = context();
            let mut bridge = make_bridge(bus_with_nodes(&[0x40, 0x41]), true, None);
            let msg = CuMsg::new(Some(JointPositions::from_slice(&[0.5, -0.25]).unwrap()));
            bridge
                .send(&ctx, &BarefocTxChannels::JOINT_POSITIONS, &msg)
                .unwrap();

            let frames = &bridge.bus.inner().frames;
            assert_eq!(frames.len(), 1);
            let mut expected = [0u8; MAX_PACKET_LEN];
            let len = build_sync_write_frame(
                REG_GOAL_POSITION,
                &[(0x40, 0.5), (0x41, -0.25)],
                &mut expected,
            )
            .unwrap();
            assert_eq!(&frames[0][..], &expected[..len]);
        });
    }

    #[test]
    fn send_velocity_uses_goal_velocity_address_and_converts_units() {
        ctx_test(|| {
            let ctx = context();
            let mut bridge = make_bridge(bus_with_nodes(&[0x40]), true, Some("deg"));
            let msg = CuMsg::new(Some(JointVelocities::from_slice(&[180.0]).unwrap()));
            bridge
                .send(&ctx, &BarefocTxChannels::JOINT_VELOCITIES, &msg)
                .unwrap();
            let frame = bridge.bus.inner().last_frame().unwrap();
            assert_eq!(&frame[8..10], &REG_GOAL_VELOCITY.to_le_bytes());
            let target = f32::from_le_bytes([frame[13], frame[14], frame[15], frame[16]]);
            assert!((target - core::f32::consts::PI).abs() < 1e-6);
        });
    }

    #[test]
    fn send_rejects_more_joints_than_nodes_without_dispatch() {
        ctx_test(|| {
            let ctx = context();
            let mut bridge = make_bridge(bus_with_nodes(&[0x40]), true, None);
            let msg = CuMsg::new(Some(JointPositions::from_slice(&[0.1, 0.2, 0.3]).unwrap()));
            let err = bridge
                .send(&ctx, &BarefocTxChannels::JOINT_POSITIONS, &msg)
                .unwrap_err();
            assert!(err.to_string().contains("capacity"));
            assert!(bridge.bus.inner().frames.is_empty());
        });
    }

    #[test]
    fn receive_publishes_cached_telemetry() {
        ctx_test(|| {
            let ctx = context();
            let mut bridge = make_bridge(bus_with_nodes(&[0x40]), false, None);
            bridge.preprocess(&ctx).unwrap();
            let mut msg = CuMsg::new(None);
            bridge
                .receive(&ctx, &BarefocRxChannels::JOINT_TELEMETRY, &mut msg)
                .unwrap();
            let payload = msg.payload().expect("telemetry payload");
            assert_eq!(payload.positions.as_slice(), &[1.0]);
        });
    }

    #[test]
    fn stop_broadcasts_emergency_disarm_frame() {
        ctx_test(|| {
            let ctx = context();
            let mut bridge = make_bridge(bus_with_nodes(&[0x40]), true, None);
            bridge.start(&ctx).unwrap();
            bridge.stop(&ctx).unwrap();
            assert!(!bridge.is_armed());
            assert!(!bridge.is_running());
            let frames = &bridge.bus.inner().frames;
            assert_eq!(frames.len(), 1);
            let frame = &frames[0][..];
            assert_eq!(&frame[..4], &barefoc_master::DXL2_HEADER);
            assert_eq!(frame[4], BROADCAST_ID);
            assert_eq!(frame[7], barefoc_master::INST_WRITE);
            assert_eq!(&frame[8..10], &REG_TORQUE_ENABLE.to_le_bytes());
            assert_eq!(frame[10], 0);
        });
    }

    #[test]
    fn messages_are_bincode_round_trippable_for_copper_logs() {
        let msg = JointTorques::from_slice(&[0.25, -0.75]).unwrap();
        let bytes = cu29::bincode::encode_to_vec(&msg, cu29::bincode::config::standard()).unwrap();
        let (decoded, _): (JointTorques, usize) =
            cu29::bincode::decode_from_slice(&bytes, cu29::bincode::config::standard()).unwrap();
        assert_eq!(decoded, msg);
        // Keeps the unused import meaningful for the contract check above.
        fn assert_contract<T: Encode + Decode<()>>() {}
        assert_contract::<JointTelemetry>();
    }

    /// Run a lifecycle test body; kept as a helper so all such tests share one
    /// deterministic host-clock construction path.
    fn ctx_test<F: FnOnce()>(f: F) {
        f();
    }
}
