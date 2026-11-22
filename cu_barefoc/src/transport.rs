//! Transport abstraction between the bridge and a BareFOC actuator bus.
//!
//! [`ActuatorBus`] is deliberately small: register reads/writes, a broadcast
//! write and one raw synchronous-dispatch frame. A [`MasterTransport`] adapter
//! implements `barefoc_master::Transport` on top of it so the SDK's
//! [`BusScanner`](barefoc_master::BusScanner), `ControlTableClient` and
//! `SyncGroup` can be reused unchanged.
//!
//! Upstream: `barefoc-master/src/transport.rs` (`Transport`, `I2cTransport`,
//! `StreamTransport`), `barefoc-master/src/sync.rs` (`SyncGroup`),
//! `barefoc-master/src/registers.rs` (`BROADCAST_ID`).

use barefoc_master::sync::MAX_PACKET_LEN;
use barefoc_master::transport::Transport;
use barefoc_master::BROADCAST_ID;
#[cfg(any(test, feature = "serial"))]
use barefoc_master::REG_TORQUE_ENABLE;

use crate::BusError;

/// Minimal register-level bus contract used by the bridge.
///
/// All methods are fallible and return [`BusError`] instead of panicking.
/// Upstream: `barefoc-master/src/transport.rs` (`Transport`).
pub trait ActuatorBus {
    /// Read exactly `out.len()` bytes starting at `reg` on `node`.
    fn read_registers(&mut self, node: u8, reg: u16, out: &mut [u8]) -> Result<(), BusError>;

    /// Write `bytes` starting at `reg` on `node`.
    fn write_registers(&mut self, node: u8, reg: u16, bytes: &[u8]) -> Result<(), BusError>;

    /// Write `bytes` to every node (broadcast identifier).
    ///
    /// The default uses the canonical broadcast ID `0xFE`.
    fn broadcast_write(&mut self, reg: u16, bytes: &[u8]) -> Result<(), BusError> {
        self.write_registers(BROADCAST_ID, reg, bytes)
    }

    /// Emit one pre-built servo-protocol frame (normally `INST_SYNC_WRITE`).
    fn send_sync_frame(&mut self, frame: &[u8]) -> Result<(), BusError>;
}

impl<B: ActuatorBus + ?Sized> ActuatorBus for &mut B {
    fn read_registers(&mut self, node: u8, reg: u16, out: &mut [u8]) -> Result<(), BusError> {
        (**self).read_registers(node, reg, out)
    }

    fn write_registers(&mut self, node: u8, reg: u16, bytes: &[u8]) -> Result<(), BusError> {
        (**self).write_registers(node, reg, bytes)
    }

    fn broadcast_write(&mut self, reg: u16, bytes: &[u8]) -> Result<(), BusError> {
        (**self).broadcast_write(reg, bytes)
    }

    fn send_sync_frame(&mut self, frame: &[u8]) -> Result<(), BusError> {
        (**self).send_sync_frame(frame)
    }
}

/// Adapts an [`ActuatorBus`] to `barefoc_master::Transport`.
///
/// Upstream: `barefoc-master/src/transport.rs` (`Transport`).
pub struct MasterTransport<B> {
    inner: B,
}

impl<B> MasterTransport<B> {
    /// Wrap an actuator bus.
    pub const fn new(inner: B) -> Self {
        Self { inner }
    }

    /// Borrow the wrapped bus.
    pub fn inner(&self) -> &B {
        &self.inner
    }

    /// Mutably borrow the wrapped bus.
    pub fn inner_mut(&mut self) -> &mut B {
        &mut self.inner
    }

    /// Consume the adapter and return the wrapped bus.
    pub fn into_inner(self) -> B {
        self.inner
    }
}

impl<B: ActuatorBus> Transport for MasterTransport<B> {
    type Error = BusError;

    fn write(
        &mut self,
        node: u8,
        reg: u16,
        bytes: &[u8],
    ) -> Result<(), barefoc_master::transport::TransportError<Self::Error>> {
        self.inner
            .write_registers(node, reg, bytes)
            .map_err(barefoc_master::transport::TransportError::Bus)
    }

    fn read(
        &mut self,
        node: u8,
        reg: u16,
        out: &mut [u8],
    ) -> Result<(), barefoc_master::transport::TransportError<Self::Error>> {
        self.inner
            .read_registers(node, reg, out)
            .map_err(barefoc_master::transport::TransportError::Bus)
    }

    fn send_packet(
        &mut self,
        packet: &[u8],
    ) -> Result<(), barefoc_master::transport::TransportError<Self::Error>> {
        self.inner
            .send_sync_frame(packet)
            .map_err(barefoc_master::transport::TransportError::Bus)
    }
}

impl<B: ActuatorBus> ActuatorBus for MasterTransport<B> {
    fn read_registers(&mut self, node: u8, reg: u16, out: &mut [u8]) -> Result<(), BusError> {
        self.inner.read_registers(node, reg, out)
    }

    fn write_registers(&mut self, node: u8, reg: u16, bytes: &[u8]) -> Result<(), BusError> {
        self.inner.write_registers(node, reg, bytes)
    }

    fn broadcast_write(&mut self, reg: u16, bytes: &[u8]) -> Result<(), BusError> {
        self.inner.broadcast_write(reg, bytes)
    }

    fn send_sync_frame(&mut self, frame: &[u8]) -> Result<(), BusError> {
        self.inner.send_sync_frame(frame)
    }
}

// ---------------------------------------------------------------------------
// In-memory mock transport
// ---------------------------------------------------------------------------

/// One recorded register write, used for byte-for-byte frame assertions.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RegisterWrite {
    /// Target node identifier (`BROADCAST_ID` for broadcasts).
    pub node: u8,
    /// Register address.
    pub reg: u16,
    /// Little-endian payload bytes.
    pub bytes: heapless::Vec<u8, MAX_PACKET_LEN>,
}

/// In-memory [`ActuatorBus`] with injectable responses and recorded traffic.
///
/// All storage is fixed capacity; constructing or using the mock never
/// allocates on the hot path. Intended for host unit tests and deterministic
/// replay tests.
#[cfg(any(test, feature = "mock"))]
#[derive(Debug, Default)]
pub struct MockBus {
    /// `(node, register, reply bytes)` response table.
    pub responses: heapless::Vec<(u8, u16, heapless::Vec<u8, 16>), 128>,
    /// Every register write issued by the bridge, in order.
    pub writes: heapless::Vec<RegisterWrite, 128>,
    /// Every raw synchronous frame issued by the bridge, in order.
    pub frames: heapless::Vec<heapless::Vec<u8, MAX_PACKET_LEN>, 16>,
    /// When set, all reads fail with [`BusError::NoAcknowledge`].
    pub fail_reads: bool,
}

#[cfg(any(test, feature = "mock"))]
impl MockBus {
    /// Create an empty mock bus.
    pub fn new() -> Self {
        Self::default()
    }

    /// Install or replace a register response.
    pub fn set_register(&mut self, node: u8, reg: u16, bytes: &[u8]) -> Result<(), BusError> {
        if let Some((_, _, reply)) = self
            .responses
            .iter_mut()
            .find(|(n, r, _)| *n == node && *r == reg)
        {
            reply.clear();
            reply
                .extend_from_slice(bytes)
                .map_err(|_| BusError::BufferTooSmall)?;
            return Ok(());
        }
        let mut reply = heapless::Vec::new();
        reply
            .extend_from_slice(bytes)
            .map_err(|_| BusError::BufferTooSmall)?;
        self.responses
            .push((node, reg, reply))
            .map_err(|_| BusError::BufferTooSmall)
    }

    /// Preload the identity fingerprint registers expected by `BusScanner`.
    ///
    /// Upstream: `barefoc-master/src/scanner.rs` (`NodeDescriptor`).
    pub fn add_node(&mut self, node: u8, model: u16, firmware: u8, uid: [u8; 12]) {
        use barefoc_master::registers::{REG_FIRMWARE_VERSION, REG_HARDWARE_UID, REG_MODEL_NUMBER};
        self.set_register(node, REG_MODEL_NUMBER, &model.to_le_bytes())
            .unwrap();
        self.set_register(node, REG_FIRMWARE_VERSION, &[firmware])
            .unwrap();
        self.set_register(node, REG_HARDWARE_UID, &uid).unwrap();
        // Capability bit 2 (temperature) and bit 0 (current sense) by default.
        self.set_register(
            node,
            barefoc_master::registers::REG_HARDWARE_CAPABILITIES,
            &(barefoc_master::registers::CAP_CURRENT_SENSE
                | barefoc_master::registers::CAP_TEMPERATURE)
                .to_le_bytes(),
        )
        .unwrap();
    }

    /// Return the most recently emitted raw frame, if any.
    pub fn last_frame(&self) -> Option<&[u8]> {
        self.frames.last().map(|frame| frame.as_slice())
    }
}

#[cfg(any(test, feature = "mock"))]
impl ActuatorBus for MockBus {
    fn read_registers(&mut self, node: u8, reg: u16, out: &mut [u8]) -> Result<(), BusError> {
        if self.fail_reads {
            return Err(BusError::NoAcknowledge);
        }
        let reply = self
            .responses
            .iter()
            .find(|(n, r, _)| *n == node && *r == reg)
            .map(|(_, _, bytes)| bytes.as_slice())
            .ok_or(BusError::NoAcknowledge)?;
        out.fill(0);
        let len = out.len().min(reply.len());
        out[..len].copy_from_slice(&reply[..len]);
        Ok(())
    }

    fn write_registers(&mut self, node: u8, reg: u16, bytes: &[u8]) -> Result<(), BusError> {
        let mut record = RegisterWrite {
            node,
            reg,
            bytes: heapless::Vec::new(),
        };
        record
            .bytes
            .extend_from_slice(bytes)
            .map_err(|_| BusError::BufferTooSmall)?;
        self.writes
            .push(record)
            .map_err(|_| BusError::BufferTooSmall)
    }

    fn broadcast_write(&mut self, reg: u16, bytes: &[u8]) -> Result<(), BusError> {
        self.write_registers(BROADCAST_ID, reg, bytes)
    }

    fn send_sync_frame(&mut self, frame: &[u8]) -> Result<(), BusError> {
        let mut record = heapless::Vec::new();
        record
            .extend_from_slice(frame)
            .map_err(|_| BusError::BufferTooSmall)?;
        self.frames
            .push(record)
            .map_err(|_| BusError::BufferTooSmall)
    }
}

// ---------------------------------------------------------------------------
// Serial transport (embedded-io, as used by cu-linux-resources)
// ---------------------------------------------------------------------------

/// Convenience alias for a serial bridge bus backed by Copper's Linux port.
///
/// Upstream: `cu-linux-resources/src/lib.rs` (`LinuxSerialPort`).
#[cfg(feature = "serial")]
pub type LinuxSerialBus = SerialBus<cu_linux_resources::LinuxSerialPort>;

/// Convenience alias for an I2C bridge bus backed by Copper's Linux I2C device.
///
/// Upstream: `cu-linux-resources/src/lib.rs` (`LinuxI2c`).
#[cfg(all(feature = "i2c", feature = "serial", target_os = "linux"))]
pub type LinuxI2cBus = I2cBus<cu_linux_resources::LinuxI2c>;

/// Serial adapter over `embedded-io` 0.7 streams.
///
/// `cu_linux_resources::LinuxSerialPort` implements both `Read` and `Write`, so
/// it can be passed directly: `SerialBus::new(port)`. The wire framing is the
/// same as `barefoc-master::StreamTransport` (`0xA5` sync, big-endian register
/// pointer, little-endian payload, XOR checksum).
///
/// Upstream: `barefoc-master/src/transport.rs` (`StreamTransport`),
/// `cu-linux-resources/src/lib.rs` (`LinuxSerialPort`).
#[cfg(feature = "serial")]
pub struct SerialBus<S> {
    io: S,
}

#[cfg(feature = "serial")]
impl<S> SerialBus<S> {
    /// Wrap a serial stream implementing both `embedded_io::Read` and `Write`.
    pub const fn new(io: S) -> Self {
        Self { io }
    }

    /// Consume the adapter and return the stream.
    pub fn into_inner(self) -> S {
        self.io
    }
}

#[cfg(feature = "serial")]
impl<S> SerialBus<S>
where
    S: embedded_io::ErrorType + embedded_io::Read + embedded_io::Write,
{
    fn exchange(
        &mut self,
        node: u8,
        reg: u16,
        payload: &[u8],
        out: &mut [u8],
    ) -> Result<(), BusError> {
        let mut request = [0u8; barefoc_master::transport::MAX_STREAM_PAYLOAD + 6];
        let len = build_stream_frame(node, reg, payload, &mut request)?;
        self.io
            .write_all(&request[..len])
            .map_err(|_| io_error::<S>())?;

        let mut header = [0u8; 5];
        self.io
            .read_exact(&mut header)
            .map_err(|e| map_exact_error::<S>(e))?;
        if header[0] != STREAM_SYNC {
            return Err(BusError::Protocol("invalid stream sync byte"));
        }
        if header[1] != node || (((header[2] as u16) << 8) | header[3] as u16) != reg {
            return Err(BusError::Protocol("stream response header mismatch"));
        }
        let reply_len = header[4] as usize;
        if reply_len != out.len() || reply_len > barefoc_master::transport::MAX_STREAM_PAYLOAD {
            return Err(BusError::Protocol("stream response length mismatch"));
        }
        self.io
            .read_exact(out)
            .map_err(|e| map_exact_error::<S>(e))?;
        let mut checksum = [0u8; 1];
        self.io
            .read_exact(&mut checksum)
            .map_err(|e| map_exact_error::<S>(e))?;

        let mut full = [0u8; barefoc_master::transport::MAX_STREAM_PAYLOAD + 5];
        full[..5].copy_from_slice(&header);
        full[5..5 + reply_len].copy_from_slice(&out[..reply_len]);
        if stream_checksum(&full[..5 + reply_len]) != checksum[0] {
            return Err(BusError::Protocol("stream response checksum mismatch"));
        }
        Ok(())
    }
}

#[cfg(feature = "serial")]
impl<S> ActuatorBus for SerialBus<S>
where
    S: embedded_io::ErrorType + embedded_io::Read + embedded_io::Write,
{
    fn read_registers(&mut self, node: u8, reg: u16, out: &mut [u8]) -> Result<(), BusError> {
        self.exchange(node, reg, &[], out)
    }

    fn write_registers(&mut self, node: u8, reg: u16, bytes: &[u8]) -> Result<(), BusError> {
        self.exchange(node, reg, bytes, &mut [])
    }

    fn send_sync_frame(&mut self, frame: &[u8]) -> Result<(), BusError> {
        self.exchange(BROADCAST_ID, REG_TORQUE_ENABLE, frame, &mut [])
    }
}

#[cfg(feature = "serial")]
const STREAM_SYNC: u8 = 0xA5;

#[cfg(feature = "serial")]
fn stream_checksum(bytes: &[u8]) -> u8 {
    bytes.iter().fold(0u8, |acc, &byte| acc ^ byte)
}

#[cfg(feature = "serial")]
fn build_stream_frame(
    node: u8,
    reg: u16,
    payload: &[u8],
    out: &mut [u8],
) -> Result<usize, BusError> {
    if payload.len() > barefoc_master::transport::MAX_STREAM_PAYLOAD
        || out.len() < payload.len() + 6
    {
        return Err(BusError::BufferTooSmall);
    }
    out[0] = STREAM_SYNC;
    out[1] = node;
    out[2] = (reg >> 8) as u8;
    out[3] = reg as u8;
    out[4] = payload.len() as u8;
    out[5..5 + payload.len()].copy_from_slice(payload);
    let end = 5 + payload.len();
    out[end] = stream_checksum(&out[..end]);
    Ok(end + 1)
}

#[cfg(feature = "serial")]
fn io_error<S: embedded_io::ErrorType>() -> BusError {
    BusError::Io(core::any::type_name::<S::Error>())
}

#[cfg(feature = "serial")]
fn map_exact_error<S: embedded_io::ErrorType>(
    err: embedded_io::ReadExactError<S::Error>,
) -> BusError {
    match err {
        embedded_io::ReadExactError::Other(_) => io_error::<S>(),
        embedded_io::ReadExactError::UnexpectedEof => {
            BusError::Protocol("unexpected EOF in stream frame")
        }
    }
}

// ---------------------------------------------------------------------------
// I2C transport (embedded-hal 1.0)
// ---------------------------------------------------------------------------

/// I2C adapter reusing `barefoc_master::I2cTransport` (two-byte big-endian
/// register pointer, little-endian payload).
///
/// On Linux `/dev/i2c-*` devices are exposed as
/// `cu_linux_resources::LinuxI2c`, which implements `embedded_hal::i2c::I2c`.
///
/// Upstream: `barefoc-master/src/transport.rs` (`I2cTransport`).
#[cfg(feature = "i2c")]
pub struct I2cBus<I> {
    transport: barefoc_master::I2cTransport<I, 18>,
}

#[cfg(feature = "i2c")]
impl<I> I2cBus<I> {
    /// Wrap an embedded-hal I2C master.
    pub const fn new(bus: I) -> Self {
        Self {
            transport: barefoc_master::I2cTransport::new(bus),
        }
    }

    /// Borrow the underlying I2C master.
    pub fn inner(&self) -> &I {
        self.transport.bus()
    }

    /// Consume the adapter and return the I2C master.
    pub fn into_inner(self) -> I {
        self.transport.into_inner()
    }
}

#[cfg(feature = "i2c")]
impl<I: embedded_hal::i2c::I2c> ActuatorBus for I2cBus<I> {
    fn read_registers(&mut self, node: u8, reg: u16, out: &mut [u8]) -> Result<(), BusError> {
        self.transport
            .read(node, reg, out)
            .map_err(map_i2c_error::<I>)
    }

    fn write_registers(&mut self, node: u8, reg: u16, bytes: &[u8]) -> Result<(), BusError> {
        self.transport
            .write(node, reg, bytes)
            .map_err(map_i2c_error::<I>)
    }

    fn send_sync_frame(&mut self, frame: &[u8]) -> Result<(), BusError> {
        self.transport
            .send_packet(frame)
            .map_err(map_i2c_error::<I>)
    }
}

#[cfg(feature = "i2c")]
fn map_i2c_error<I: embedded_hal::i2c::I2c>(
    err: barefoc_master::transport::TransportError<I::Error>,
) -> BusError {
    use barefoc_master::transport::TransportError;
    match err {
        TransportError::Bus(_) => BusError::Io(core::any::type_name::<I::Error>()),
        TransportError::Timeout => BusError::Timeout,
        TransportError::NoAcknowledge => BusError::NoAcknowledge,
        TransportError::BufferTooSmall => BusError::BufferTooSmall,
        TransportError::InvalidAddress(addr) => BusError::InvalidAddress(addr),
        TransportError::Protocol(msg) => BusError::Protocol(msg),
    }
}

// ---------------------------------------------------------------------------
// CAN / CAN-FD frame adapter
// ---------------------------------------------------------------------------

/// Payload capacity of one classic CAN-FD data frame.
#[cfg(feature = "can")]
pub const CAN_FD_PAYLOAD_LEN: usize = 64;

/// One CAN-FD frame produced or consumed by [`CanBus`].
#[cfg(feature = "can")]
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CanFrame {
    /// CAN identifier (11-bit or 29-bit, as configured by the caller).
    pub id: u32,
    /// Frame payload, at most [`CAN_FD_PAYLOAD_LEN`] bytes.
    pub data: heapless::Vec<u8, CAN_FD_PAYLOAD_LEN>,
}

/// SocketCAN-oriented frame adapter.
///
/// Register accesses are delegated to an inner [`ActuatorBus`] (typically an
/// [`I2cBus`] or [`MockBus`]). Raw `INST_SYNC_WRITE` frames are fragmented into
/// CAN-FD data frames with a four-byte header `[index, total, length, 0]` and
/// queued in `tx_frames`; the application drains them into a SocketCAN socket.
///
/// # Platform limitation
///
/// This crate deliberately does not bind `AF_CAN` directly: doing so needs an
/// extra OS crate (`socketcan`) and raw FFI, which the project's `unsafe_code =
/// forbid` policy excludes. The adapter is therefore transport-agnostic and
/// hands complete CAN frames to the surrounding Copper resource layer. On
/// Linux that layer can use `socketcan::CanSocket::write_frame` with the
/// frames returned by [`Self::drain_tx_frames`].
///
/// Upstream: `barefoc-master/src/sync.rs` (`MAX_PACKET_LEN`, `INST_SYNC_WRITE`).
#[cfg(feature = "can")]
pub struct CanBus<B> {
    inner: B,
    can_id_base: u32,
    tx_frames: heapless::Vec<CanFrame, 16>,
}

#[cfg(feature = "can")]
impl<B> CanBus<B> {
    /// Wrap `inner` and allocate CAN identifiers starting at `can_id_base`.
    pub const fn new(inner: B, can_id_base: u32) -> Self {
        Self {
            inner,
            can_id_base,
            tx_frames: heapless::Vec::new(),
        }
    }

    /// Remove and return all queued transmit frames.
    pub fn drain_tx_frames(&mut self) -> heapless::Vec<CanFrame, 16> {
        core::mem::take(&mut self.tx_frames)
    }

    /// Borrow the inner register bus.
    pub fn inner(&self) -> &B {
        &self.inner
    }

    /// Mutably borrow the inner register bus.
    pub fn inner_mut(&mut self) -> &mut B {
        &mut self.inner
    }
}

#[cfg(feature = "can")]
impl<B: ActuatorBus> ActuatorBus for CanBus<B> {
    fn read_registers(&mut self, node: u8, reg: u16, out: &mut [u8]) -> Result<(), BusError> {
        self.inner.read_registers(node, reg, out)
    }

    fn write_registers(&mut self, node: u8, reg: u16, bytes: &[u8]) -> Result<(), BusError> {
        self.inner.write_registers(node, reg, bytes)
    }

    fn broadcast_write(&mut self, reg: u16, bytes: &[u8]) -> Result<(), BusError> {
        self.inner.broadcast_write(reg, bytes)
    }

    fn send_sync_frame(&mut self, frame: &[u8]) -> Result<(), BusError> {
        const CHUNK: usize = CAN_FD_PAYLOAD_LEN - 4;
        let total = frame.len().div_ceil(CHUNK).max(1);
        if total > u8::MAX as usize {
            return Err(BusError::BufferTooSmall);
        }
        for (index, chunk) in frame.chunks(CHUNK).enumerate() {
            let mut data = heapless::Vec::new();
            let header = [index as u8, total as u8, chunk.len() as u8, 0u8];
            data.extend_from_slice(&header)
                .map_err(|_| BusError::BufferTooSmall)?;
            data.extend_from_slice(chunk)
                .map_err(|_| BusError::BufferTooSmall)?;
            self.tx_frames
                .push(CanFrame {
                    id: self.can_id_base + index as u32,
                    data,
                })
                .map_err(|_| BusError::BufferTooSmall)?;
        }
        Ok(())
    }
}

#[cfg(all(test, feature = "mock"))]
mod tests {
    use super::*;

    #[test]
    fn mock_records_exact_register_writes_and_frames() {
        let mut bus = MockBus::new();
        bus.write_registers(0x40, REG_TORQUE_ENABLE, &[0x01])
            .unwrap();
        bus.broadcast_write(REG_TORQUE_ENABLE, &[0x00]).unwrap();
        bus.send_sync_frame(&[0xFF, 0xFF, 0xFD, 0x00]).unwrap();

        assert_eq!(bus.writes.len(), 2);
        assert_eq!(bus.writes[0].node, 0x40);
        assert_eq!(bus.writes[0].reg, REG_TORQUE_ENABLE);
        assert_eq!(&bus.writes[0].bytes[..], &[0x01]);
        assert_eq!(bus.writes[1].node, BROADCAST_ID);
        assert_eq!(&bus.writes[1].bytes[..], &[0x00]);
        assert_eq!(bus.last_frame(), Some(&[0xFF, 0xFF, 0xFD, 0x00][..]));
    }

    #[test]
    fn mock_injects_read_responses_and_fails_closed() {
        let mut bus = MockBus::new();
        bus.set_register(0x40, 0x144, &1.5f32.to_le_bytes())
            .unwrap();
        let mut out = [0u8; 4];
        bus.read_registers(0x40, 0x144, &mut out).unwrap();
        assert_eq!(f32::from_le_bytes(out), 1.5);
        assert!(bus.read_registers(0x41, 0x144, &mut out).is_err());
        bus.fail_reads = true;
        assert!(bus.read_registers(0x40, 0x144, &mut out).is_err());
    }
}

#[cfg(all(test, feature = "can"))]
mod can_tests {
    use super::*;

    #[test]
    fn can_bus_fragments_sync_frame_and_delegates_registers() {
        let mut inner = MockBus::new();
        inner.add_node(0x40, 0x0503, 7, [0x11; 12]);
        let mut bus = CanBus::new(inner, 0x100);
        let mut out = [0u8; 2];
        bus.read_registers(0x40, barefoc_master::REG_MODEL_NUMBER, &mut out)
            .unwrap();
        let frame = [0xABu8; 100];
        bus.send_sync_frame(&frame).unwrap();
        let frames = bus.drain_tx_frames();
        assert_eq!(frames.len(), 2);
        assert_eq!(frames[0].id, 0x100);
        assert_eq!(frames[1].id, 0x101);
        assert_eq!(frames[0].data[0], 0);
        assert_eq!(frames[0].data[1], 2);
        assert_eq!(frames[0].data[2], 60);
        assert_eq!(&frames[0].data[4..], &frame[..60]);
        assert_eq!(&frames[1].data[4..], &frame[60..]);
        assert!(bus.drain_tx_frames().is_empty());
    }
}
