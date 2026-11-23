//! Downstream bus abstraction and the concrete `BusRouter` dispatch enum.
//!
//! The server never talks to a peripheral directly. [`BusAccess`] is the
//! minimal seam the application implements; [`DeviceBuses`] is the ready-made
//! implementation that reuses [`barefoc_master::I2cTransport`] and
//! [`barefoc_master::StreamTransport`] as inner transports, so register
//! semantics are identical whether the host is local or remote.
//!
//! Upstream: `barefoc-comms/barefoc-multibus/src/transport.rs`
//! (`TargetTransport` / `RegServer` single-task event model),
//! `barefoc-master/src/transport.rs` (inner transport adapters).

use barefoc_master::{
    bus_kind, scanner::{I2C_SCAN_END, I2C_SCAN_START}, BusKind, I2cTransport, StatusCode,
    StreamTransport, Transport, TransportError, BROADCAST_ID, BUS_CAP_I2C, BUS_CAP_LOCAL,
    BUS_CAP_UART, BUS_LOCAL, LOCAL_REG_CAPABILITIES, LOCAL_REG_ERROR_FLAGS, LOCAL_REG_FIRMWARE_VERSION,
    LOCAL_REG_MODE_STATE, LOCAL_REG_TEMPERATURE, LOCAL_REG_WATCHDOG_TIMEOUT_MS, REG_TORQUE_ENABLE,
};
use embedded_hal::i2c::I2c;
use embedded_io::{ErrorType as IoErrorType, Read as IoRead, Write as IoWrite};

/// Map an inner [`Transport`] failure to a wire [`StatusCode`].
fn inner_status<E>(result: Result<(), TransportError<E>>) -> Result<(), StatusCode> {
    match result {
        Ok(()) => Ok(()),
        Err(TransportError::NoAcknowledge) => Err(StatusCode::BusNak),
        Err(TransportError::Timeout) => Err(StatusCode::BusTimeout),
        Err(TransportError::BufferTooSmall) | Err(TransportError::InvalidAddress(_)) => {
            Err(StatusCode::InvalidLength)
        }
        Err(TransportError::Bus(_)) => Err(StatusCode::BusNak),
        Err(TransportError::Protocol(_)) => Err(StatusCode::ExecutionFault),
    }
}

/// Minimal device-side seam between the passthrough server and the bus owner.
///
/// A single application task owns bus I/O while passthrough is active; all
/// methods are called only from that task (compare the `TargetTransport`
/// event-draining contract in `barefoc-multibus`).
pub trait BusAccess {
    /// Bitmask of routed bus classes, built from `BUS_CAP_*`.
    fn supported_buses(&self) -> u32;

    /// Flight-controller firmware version reported by `GET_INFO`.
    fn firmware_version(&self) -> u32;

    /// Read from the flight controller's local control table (bus `0xF0`).
    fn local_read(&mut self, reg: u16, out: &mut [u8]) -> Result<(), StatusCode>;

    /// Write to the flight controller's local control table (bus `0xF0`).
    fn local_write(&mut self, reg: u16, bytes: &[u8]) -> Result<(), StatusCode>;

    /// Write `bytes` to `reg` of `node` on `bus`.
    fn write(
        &mut self,
        bus: u8,
        node: u8,
        reg: u16,
        bytes: &[u8],
    ) -> Result<(), StatusCode>;

    /// Read `out.len()` bytes from `reg` of `node` on `bus`.
    fn read(&mut self, bus: u8, node: u8, reg: u16, out: &mut [u8]) -> Result<(), StatusCode>;

    /// Dispatch a prebuilt raw packet as one atomic bus transaction.
    fn send_packet(&mut self, bus: u8, packet: &[u8]) -> Result<(), StatusCode>;

    /// Fill `mask` with responding 7-bit I2C addresses on `bus`.
    fn scan_i2c(&mut self, bus: u8, mask: &mut [u8; 16]) -> Result<(), StatusCode>;

    /// Broadcast `REG_TORQUE_ENABLE = 0` to every actuator bus.
    fn broadcast_disarm(&mut self) -> Result<(), StatusCode>;

    /// True only when the FC and every downstream node report torque disabled.
    fn all_disarmed(&mut self) -> bool;

    /// Called on mode transitions so the application can suspend or resume
    /// normal flight-control bus access and restore bus roles.
    fn set_passthrough_active(&mut self, active: bool);
}

/// Flight-controller-local control table exposed on bus `0xF0`.
///
/// Upstream: `bareFOC/src/control_table.rs` (identity / status registers),
/// `openspec/changes/bus-passthrough/design.md` (decision 5).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LocalControlTable {
    mode_state: u8,
    firmware_version: u32,
    capabilities: u32,
    error_flags: u32,
    temperature_c: f32,
    watchdog_ms: u16,
    fc_armed: bool,
}

impl LocalControlTable {
    /// Create a local table with application-supplied identity values.
    pub const fn new(firmware_version: u32, capabilities: u32) -> Self {
        Self {
            mode_state: 0,
            firmware_version,
            capabilities,
            error_flags: 0,
            temperature_c: 0.0,
            watchdog_ms: crate::server::DEFAULT_WATCHDOG_MS,
            fc_armed: false,
        }
    }

    /// Current mode state byte (`0` normal, `1` passthrough).
    pub const fn mode_state(&self) -> u8 {
        self.mode_state
    }

    /// Set the mode state byte (called by the safety FSM).
    pub fn set_mode_state(&mut self, active: bool) {
        self.mode_state = if active { 1 } else { 0 };
    }

    /// FC arm state; `true` blocks passthrough entry.
    pub const fn fc_armed(&self) -> bool {
        self.fc_armed
    }

    /// Set the FC arm state.
    pub fn set_fc_armed(&mut self, armed: bool) {
        self.fc_armed = armed;
    }

    /// Publish thermal telemetry in degrees Celsius.
    pub fn set_temperature_c(&mut self, celsius: f32) {
        self.temperature_c = celsius;
    }

    /// Publish the FC error bitmask.
    pub fn set_error_flags(&mut self, flags: u32) {
        self.error_flags = flags;
    }

    /// Configure the watchdog timeout (milliseconds); zero reverts to default.
    pub fn set_watchdog_ms(&mut self, ms: u16) {
        self.watchdog_ms = if ms == 0 {
            crate::server::DEFAULT_WATCHDOG_MS
        } else {
            ms
        };
    }

    /// Currently configured watchdog timeout.
    pub const fn watchdog_ms(&self) -> u16 {
        self.watchdog_ms
    }

    /// Read a local control-table register.
    pub fn read(&self, reg: u16, out: &mut [u8]) -> Result<(), StatusCode> {
        match reg {
            LOCAL_REG_MODE_STATE => {
                if out.is_empty() {
                    return Err(StatusCode::InvalidLength);
                }
                out[0] = self.mode_state;
            }
            LOCAL_REG_FIRMWARE_VERSION => {
                if out.len() < 4 {
                    return Err(StatusCode::InvalidLength);
                }
                out[..4].copy_from_slice(&self.firmware_version.to_le_bytes());
            }
            LOCAL_REG_CAPABILITIES => {
                if out.len() < 4 {
                    return Err(StatusCode::InvalidLength);
                }
                out[..4].copy_from_slice(&self.capabilities.to_le_bytes());
            }
            LOCAL_REG_ERROR_FLAGS => {
                if out.len() < 4 {
                    return Err(StatusCode::InvalidLength);
                }
                out[..4].copy_from_slice(&self.error_flags.to_le_bytes());
            }
            LOCAL_REG_TEMPERATURE => {
                if out.len() < 4 {
                    return Err(StatusCode::InvalidLength);
                }
                out[..4].copy_from_slice(&self.temperature_c.to_le_bytes());
            }
            LOCAL_REG_WATCHDOG_TIMEOUT_MS => {
                if out.len() < 2 {
                    return Err(StatusCode::InvalidLength);
                }
                out[..2].copy_from_slice(&self.watchdog_ms.to_le_bytes());
            }
            _ => return Err(StatusCode::InvalidBus),
        }
        Ok(())
    }

    /// Write a local control-table register. Only the watchdog timeout is
    /// host-writable; mode state is owned by the safety FSM.
    pub fn write(&mut self, reg: u16, bytes: &[u8]) -> Result<(), StatusCode> {
        match reg {
            LOCAL_REG_WATCHDOG_TIMEOUT_MS => {
                if bytes.len() < 2 {
                    return Err(StatusCode::InvalidLength);
                }
                self.set_watchdog_ms(u16::from_le_bytes([bytes[0], bytes[1]]));
            }
            _ => return Err(StatusCode::InvalidBus),
        }
        Ok(())
    }
}

/// Small dispatch enum selecting one inner transport by bus ID.
///
/// No trait objects and no allocation: the application supplies the concrete
/// buses once and [`DeviceBuses::route`] borrows the selected one for the
/// duration of a single frame.
pub enum BusRouter<'a, I, RX, TX> {
    /// I2C master (`0x00..=0x0F`).
    I2c(&'a mut I2cTransport<I>),
    /// Framed register-protocol UART master (`0x10..=0x1F`).
    Uart(&'a mut StreamTransport<RX, TX>),
    /// Flight-controller-local control table (`0xF0`).
    Local(&'a mut LocalControlTable),
}

impl<I, RX, TX> BusRouter<'_, I, RX, TX>
where
    I: I2c,
    RX: IoRead + IoErrorType,
    TX: IoWrite + IoErrorType,
{
    /// Forward a register write to the selected inner transport.
    pub fn write(
        &mut self,
        node: u8,
        reg: u16,
        bytes: &[u8],
    ) -> Result<(), StatusCode> {
        match self {
            Self::I2c(t) => inner_status(t.write(node, reg, bytes)),
            Self::Uart(t) => inner_status(t.write(node, reg, bytes)),
            Self::Local(t) => t.write(reg, bytes),
        }
    }

    /// Forward a register read to the selected inner transport.
    pub fn read(&mut self, node: u8, reg: u16, out: &mut [u8]) -> Result<(), StatusCode> {
        match self {
            Self::I2c(t) => inner_status(t.read(node, reg, out)),
            Self::Uart(t) => inner_status(t.read(node, reg, out)),
            Self::Local(t) => t.read(reg, out),
        }
    }

    /// Dispatch a raw packet as a single atomic transaction.
    pub fn send_packet(&mut self, _node: u8, packet: &[u8]) -> Result<(), StatusCode> {
        match self {
            Self::I2c(t) => inner_status(t.send_packet(packet)),
            Self::Uart(t) => inner_status(t.send_packet(packet)),
            Self::Local(_) => Err(StatusCode::InvalidBus),
        }
    }

    /// Probe all 7-bit I2C addresses and fill the 16-byte bitmask.
    pub fn scan_i2c(&mut self, mask: &mut [u8; 16]) -> Result<(), StatusCode> {
        match self {
            Self::I2c(t) => {
                for address in I2C_SCAN_START..=I2C_SCAN_END {
                    let mut probe = [0u8; 2];
                    if t.read(address, barefoc_master::REG_MODEL_NUMBER, &mut probe).is_ok() {
                        mask[(address / 8) as usize] |= 1 << (address % 8);
                    }
                }
                Ok(())
            }
            _ => Err(StatusCode::InvalidBus),
        }
    }
}

/// Ready-made bus set with optional I2C and UART downstream transports.
///
/// CAN buses are reserved in protocol v1; applications that need them can
/// implement [`BusAccess`] directly and route `0x20..=0x2F` to their own
/// adapter. The local control table is always present.
pub struct DeviceBuses<I, RX, TX> {
    i2c: Option<I2cTransport<I>>,
    uart: Option<StreamTransport<RX, TX>>,
    local: LocalControlTable,
}

impl<I, RX, TX> DeviceBuses<I, RX, TX> {
    /// Assemble a bus set from optional inner transports.
    pub const fn new(
        i2c: Option<I2cTransport<I>>,
        uart: Option<StreamTransport<RX, TX>>,
        local: LocalControlTable,
    ) -> Self {
        Self { i2c, uart, local }
    }

    /// Borrow the local flight-controller control table.
    pub const fn local(&self) -> &LocalControlTable {
        &self.local
    }

    /// Mutably borrow the local flight-controller control table.
    pub fn local_mut(&mut self) -> &mut LocalControlTable {
        &mut self.local
    }

    /// Select the inner transport for `bus`.
    pub fn route(&mut self, bus: u8) -> Result<BusRouter<'_, I, RX, TX>, StatusCode> {
        match bus_kind(bus) {
            BusKind::I2c => self
                .i2c
                .as_mut()
                .map(BusRouter::I2c)
                .ok_or(StatusCode::InvalidBus),
            BusKind::Uart => self
                .uart
                .as_mut()
                .map(BusRouter::Uart)
                .ok_or(StatusCode::InvalidBus),
            BusKind::Local => Ok(BusRouter::Local(&mut self.local)),
            _ => Err(StatusCode::InvalidBus),
        }
    }
}

impl<I, RX, TX> BusAccess for DeviceBuses<I, RX, TX>
where
    I: I2c,
    RX: IoRead + IoErrorType,
    TX: IoWrite + IoErrorType,
{
    fn supported_buses(&self) -> u32 {
        let mut mask = BUS_CAP_LOCAL;
        if self.i2c.is_some() {
            mask |= BUS_CAP_I2C;
        }
        if self.uart.is_some() {
            mask |= BUS_CAP_UART;
        }
        mask
    }

    fn firmware_version(&self) -> u32 {
        self.local.firmware_version
    }

    fn local_read(&mut self, reg: u16, out: &mut [u8]) -> Result<(), StatusCode> {
        self.local.read(reg, out)
    }

    fn local_write(&mut self, reg: u16, bytes: &[u8]) -> Result<(), StatusCode> {
        self.local.write(reg, bytes)
    }

    fn write(
        &mut self,
        bus: u8,
        node: u8,
        reg: u16,
        bytes: &[u8],
    ) -> Result<(), StatusCode> {
        self.route(bus)?.write(node, reg, bytes)
    }

    fn read(&mut self, bus: u8, node: u8, reg: u16, out: &mut [u8]) -> Result<(), StatusCode> {
        self.route(bus)?.read(node, reg, out)
    }

    fn send_packet(&mut self, bus: u8, packet: &[u8]) -> Result<(), StatusCode> {
        let node = if bus == BUS_LOCAL { 0 } else { BROADCAST_ID };
        self.route(bus)?.send_packet(node, packet)
    }

    fn scan_i2c(&mut self, bus: u8, mask: &mut [u8; 16]) -> Result<(), StatusCode> {
        self.route(bus)?.scan_i2c(mask)
    }

    fn broadcast_disarm(&mut self) -> Result<(), StatusCode> {
        let disarm = [0u8];
        let mut status = StatusCode::Ok;
        if let Some(i2c) = self.i2c.as_mut() {
            if inner_status(i2c.write(BROADCAST_ID, REG_TORQUE_ENABLE, &disarm)).is_err() {
                status = StatusCode::BusNak;
            }
        }
        if let Some(uart) = self.uart.as_mut() {
            if inner_status(uart.write(BROADCAST_ID, REG_TORQUE_ENABLE, &disarm)).is_err() {
                status = StatusCode::BusNak;
            }
        }
        // A local FC never exposes its own motor bridge on this bus; mark it
        // disarmed as part of the transition so a later entry is permitted.
        self.local.set_fc_armed(false);
        if status == StatusCode::Ok {
            Ok(())
        } else {
            Err(status)
        }
    }

    fn all_disarmed(&mut self) -> bool {
        if self.local.fc_armed() {
            return false;
        }
        let mut byte = [0u8];
        if let Some(i2c) = self.i2c.as_mut() {
            for address in I2C_SCAN_START..=I2C_SCAN_END {
                if i2c.read(address, REG_TORQUE_ENABLE, &mut byte).is_ok() && byte[0] != 0 {
                    return false;
                }
            }
        }
        if let Some(uart) = self.uart.as_mut() {
            for node in 1u8..=0x7E {
                if uart.read(node, REG_TORQUE_ENABLE, &mut byte).is_ok() && byte[0] != 0 {
                    return false;
                }
            }
        }
        true
    }

    fn set_passthrough_active(&mut self, active: bool) {
        self.local.set_mode_state(active);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use barefoc_master::{
        BUS_CAN_BASE, BUS_I2C_BASE, BUS_I2C_LAST, BUS_UART_BASE, BUS_UART_LAST,
    };

    #[test]
    fn local_table_exposes_identity_and_mode() {
        let mut local = LocalControlTable::new(0x0002_0007, 0x0B);
        assert_eq!(local.mode_state(), 0);
        local.set_mode_state(true);
        assert_eq!(local.mode_state(), 1);

        let mut out = [0u8; 4];
        local.read(LOCAL_REG_MODE_STATE, &mut out).unwrap();
        assert_eq!(out[0], 1);
        local.read(LOCAL_REG_FIRMWARE_VERSION, &mut out).unwrap();
        assert_eq!(u32::from_le_bytes(out), 0x0002_0007);
        local.read(LOCAL_REG_CAPABILITIES, &mut out).unwrap();
        assert_eq!(u32::from_le_bytes(out), 0x0B);
        assert_eq!(local.read(0x1234, &mut out), Err(StatusCode::InvalidBus));

        local.write(LOCAL_REG_WATCHDOG_TIMEOUT_MS, &750u16.to_le_bytes()).unwrap();
        assert_eq!(local.watchdog_ms(), 750);
        local.read(LOCAL_REG_WATCHDOG_TIMEOUT_MS, &mut out).unwrap();
        assert_eq!(u16::from_le_bytes([out[0], out[1]]), 750);
    }

    #[test]
    fn bus_kind_constants_are_partitioned() {
        assert_eq!(bus_kind(BUS_I2C_BASE), BusKind::I2c);
        assert_eq!(bus_kind(BUS_I2C_LAST), BusKind::I2c);
        assert_eq!(bus_kind(BUS_UART_BASE), BusKind::Uart);
        assert_eq!(bus_kind(BUS_UART_LAST), BusKind::Uart);
        assert_eq!(bus_kind(BUS_CAN_BASE), BusKind::Can);
        assert_eq!(bus_kind(BUS_LOCAL), BusKind::Local);
    }
}
