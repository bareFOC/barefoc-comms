//! Safety-gated passthrough server and framing state machine.
//!
//! `NORMAL --OP_ENTER(disarmed)--> PASSTHROUGH --OP_EXIT / watchdog / link loss--> NORMAL`.
//! Every exit broadcasts `REG_TORQUE_ENABLE = 0` on all buses and restores the
//! application's normal bus roles through [`BusAccess::set_passthrough_active`].
//!
//! Upstream: `barefoc-comms/barefoc-multibus/src/transport.rs` (single task
//! drains bus events), `barefoc-master/src/passthrough.rs` (shared frame
//! codec), `openspec/changes/bus-passthrough/design.md` (decisions 2 and 6).

use barefoc_master::{
    bus_kind, decode_frame, encode_frame, response_opcode, BusKind, DeviceInfo, Frame, FrameError,
    StatusCode, BUS_LOCAL, BUS_CAP_LOCAL, LOCAL_REG_WATCHDOG_TIMEOUT_MS, MAX_FRAME_LEN, OP_ENTER,
    OP_EXIT, OP_GET_INFO, OP_KEEPALIVE, OP_PING, OP_READ, OP_SCAN_BUS, OP_SYNC_WRITE, OP_WRITE,
    OP_EMERGENCY_STOP, PROTOCOL_VERSION, PT_HEADER_LEN, PT_MAX_PAYLOAD, PT_OVERHEAD, PT_SYNC,
    RESP_EXIT, SCAN_MASK_LEN,
};
use embedded_io::{Error as IoError, ErrorKind as IoErrorKind, Read as IoRead, Write as IoWrite};

use crate::buses::BusAccess;

pub use barefoc_master::DEFAULT_WATCHDOG_MS;

/// Reduce a bus result to the wire status code (success becomes `Ok`).
fn status_of(result: Result<(), StatusCode>) -> StatusCode {
    match result {
        Ok(()) => StatusCode::Ok,
        Err(status) => status,
    }
}

/// Passthrough safety state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    /// Normal flight-control operation; only `PING`/`GET_INFO`/`ENTER` accepted.
    Normal,
    /// Passthrough owns all bus I/O; register frames are forwarded.
    Passthrough,
}

/// Errors surfaced by the device server.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ServerError {
    /// Underlying byte-stream I/O error kind.
    Io(IoErrorKind),
    /// The link closed while reading a frame (treated as link loss).
    UnexpectedEof,
    /// Frame codec rejection (reported as a status response where possible).
    Frame(FrameError),
    /// Caller-provided response buffer is too small.
    BufferTooSmall,
}

fn map_exact_error<E: IoError>(error: embedded_io::ReadExactError<E>) -> ServerError {
    match error {
        embedded_io::ReadExactError::Other(err) => ServerError::Io(err.kind()),
        embedded_io::ReadExactError::UnexpectedEof => ServerError::UnexpectedEof,
    }
}

/// Device-side passthrough server, generic over a blocking byte stream and a
/// bus owner.
///
/// The same instance serves native USB CDC, USART CDC, RTT or any other
/// `embedded_io` stream; the link choice is application wiring only.
pub struct PassthroughServer<R, W, B> {
    reader: R,
    writer: W,
    buses: B,
    mode: Mode,
    watchdog_ms: u16,
    last_activity_ms: u32,
}

impl<R, W, B> PassthroughServer<R, W, B> {
    /// Construct a server in [`Mode::Normal`] with the default watchdog.
    pub const fn new(reader: R, writer: W, buses: B) -> Self {
        Self {
            reader,
            writer,
            buses,
            mode: Mode::Normal,
            watchdog_ms: DEFAULT_WATCHDOG_MS,
            last_activity_ms: 0,
        }
    }

    /// Override the watchdog timeout (milliseconds).
    pub fn with_watchdog_ms(mut self, ms: u16) -> Self {
        self.watchdog_ms = if ms == 0 { DEFAULT_WATCHDOG_MS } else { ms };
        self
    }

    /// Current safety state.
    pub const fn mode(&self) -> Mode {
        self.mode
    }

    /// True while passthrough owns the buses.
    pub const fn is_active(&self) -> bool {
        matches!(self.mode, Mode::Passthrough)
    }

    /// Configured watchdog timeout in milliseconds.
    pub const fn watchdog_ms(&self) -> u16 {
        self.watchdog_ms
    }

    /// Set the watchdog timeout in milliseconds.
    pub fn set_watchdog_ms(&mut self, ms: u16) {
        self.watchdog_ms = if ms == 0 { DEFAULT_WATCHDOG_MS } else { ms };
    }

    /// Borrow the bus owner.
    pub const fn buses(&self) -> &B {
        &self.buses
    }

    /// Mutably borrow the bus owner (application telemetry publication).
    pub fn buses_mut(&mut self) -> &mut B {
        &mut self.buses
    }

    /// Borrow the stream halves.
    pub const fn streams(&self) -> (&R, &W) {
        (&self.reader, &self.writer)
    }

    /// Consume the server and return its parts.
    pub fn into_parts(self) -> (R, W, B) {
        (self.reader, self.writer, self.buses)
    }

    /// Leave passthrough, broadcast a disarm and restore normal bus roles.
    fn exit(&mut self) -> StatusCode where B: BusAccess {
        let status = status_of(self.buses.broadcast_disarm());
        self.mode = Mode::Normal;
        self.buses.set_passthrough_active(false);
        status
    }

    /// Force an immediate safety exit (used for link loss and faults).
    pub fn force_exit(&mut self) -> StatusCode
    where
        B: BusAccess,
    {
        self.exit()
    }

    fn write_response(
        &self,
        out: &mut [u8],
        opcode: u8,
        status: StatusCode,
        payload: &[u8],
    ) -> Result<usize, ServerError> {
        encode_frame(
            &Frame::response(opcode, BUS_LOCAL, 0, status, 0, payload),
            out,
        )
        .map_err(|e| match e {
            FrameError::BufferTooSmall => ServerError::BufferTooSmall,
            other => ServerError::Frame(other),
        })
    }

    /// Decode one frame and produce its response, without touching the stream.
    ///
    /// A malformed frame is rejected with an error status but does **not**
    /// change the safety state. Only valid frames refresh the watchdog.
    pub fn handle_frame(
        &mut self,
        bytes: &[u8],
        now_ms: u32,
        out: &mut [u8],
    ) -> Result<usize, ServerError>
    where
        B: BusAccess,
    {
        let echoed_opcode = bytes.get(2).copied().unwrap_or(0);
        let frame = match decode_frame(bytes) {
            Ok(frame) => frame,
            Err(error) => {
                return self.write_response(
                    out,
                    response_opcode(echoed_opcode),
                    error.status(),
                    &[],
                );
            }
        };

        if self.mode == Mode::Normal && requires_passthrough(frame.opcode) {
            return self.write_response(
                out,
                response_opcode(frame.opcode),
                StatusCode::NotInPassthrough,
                &[],
            );
        }

        self.last_activity_ms = now_ms;
        let mut payload = [0u8; PT_MAX_PAYLOAD];
        let (status, len) = self.dispatch(&frame, now_ms, &mut payload);
        self.write_response(out, frame.opcode, status, &payload[..len])
    }

    fn dispatch(
        &mut self,
        frame: &Frame<'_>,
        now_ms: u32,
        payload: &mut [u8; PT_MAX_PAYLOAD],
    ) -> (StatusCode, usize)
    where
        B: BusAccess,
    {
        match frame.opcode {
            OP_PING | OP_KEEPALIVE => (StatusCode::Ok, 0),
            OP_ENTER => {
                if !self.buses.all_disarmed() {
                    return (StatusCode::NotDisarmed, 0);
                }
                let status = status_of(self.buses.broadcast_disarm());
                if status == StatusCode::Ok {
                    self.mode = Mode::Passthrough;
                    self.buses.set_passthrough_active(true);
                    self.last_activity_ms = now_ms;
                }
                (status, 0)
            }
            OP_EXIT | OP_EMERGENCY_STOP => (self.exit(), 0),
            OP_GET_INFO => {
                let info = DeviceInfo {
                    protocol_version: PROTOCOL_VERSION,
                    supported_buses: self.buses.supported_buses() | BUS_CAP_LOCAL,
                    max_payload: PT_MAX_PAYLOAD as u16,
                    firmware_version: self.buses.firmware_version(),
                };
                match info.encode(payload) {
                    Ok(len) => (StatusCode::Ok, len),
                    Err(_) => (StatusCode::ExecutionFault, 0),
                }
            }
            OP_READ => {
                let len = frame.payload.len();
                if len == 0 || len > PT_MAX_PAYLOAD {
                    return (StatusCode::InvalidLength, 0);
                }
                let status = status_of(self.buses.read(
                    frame.bus,
                    frame.node,
                    frame.register(),
                    &mut payload[..len],
                ));
                (status, if status == StatusCode::Ok { len } else { 0 })
            }
            OP_WRITE => {
                let status = status_of(self.buses.write(
                    frame.bus,
                    frame.node,
                    frame.register(),
                    frame.payload,
                ));
                if status == StatusCode::Ok
                    && frame.bus == BUS_LOCAL
                    && frame.register() == LOCAL_REG_WATCHDOG_TIMEOUT_MS
                    && frame.payload.len() >= 2
                {
                    self.set_watchdog_ms(u16::from_le_bytes([
                        frame.payload[0],
                        frame.payload[1],
                    ]));
                }
                (status, 0)
            }
            OP_SYNC_WRITE => (
                status_of(self.buses.send_packet(frame.bus, frame.payload)),
                0,
            ),
            OP_SCAN_BUS => {
                if bus_kind(frame.bus) != BusKind::I2c {
                    return (StatusCode::InvalidBus, 0);
                }
                let mut mask = [0u8; SCAN_MASK_LEN];
                let status = status_of(self.buses.scan_i2c(frame.bus, &mut mask));
                if status != StatusCode::Ok {
                    return (status, 0);
                }
                payload[..SCAN_MASK_LEN].copy_from_slice(&mask);
                (StatusCode::Ok, SCAN_MASK_LEN)
            }
            _ => (StatusCode::UnsupportedOpcode, 0),
        }
    }

    /// Check the watchdog and emit an `OP_EXIT` response when it expired.
    pub fn poll(
        &mut self,
        now_ms: u32,
        out: &mut [u8],
    ) -> Result<Option<usize>, ServerError>
    where
        B: BusAccess,
    {
        if self.is_active()
            && now_ms.wrapping_sub(self.last_activity_ms) >= self.watchdog_ms as u32
        {
            let status = self.exit();
            return self
                .write_response(out, RESP_EXIT, status, &[])
                .map(Some);
        }
        Ok(None)
    }

    fn read_frame(&mut self, buf: &mut [u8; MAX_FRAME_LEN]) -> Result<usize, ServerError>
    where
        R: IoRead,
    {
        // Resynchronise on the sync byte so a stray CDC byte cannot wedge the
        // link; the CRC still rejects any resulting malformed frame.
        loop {
            let mut sync = [0u8; 1];
            self.reader
                .read_exact(&mut sync)
                .map_err(map_exact_error)?;
            if sync[0] == PT_SYNC {
                buf[0] = sync[0];
                break;
            }
        }
        self.reader
            .read_exact(&mut buf[1..PT_HEADER_LEN])
            .map_err(map_exact_error)?;
        let len = buf[7] as usize;
        if len > PT_MAX_PAYLOAD {
            return Err(ServerError::Frame(FrameError::PayloadTooLarge));
        }
        let total = PT_OVERHEAD + len;
        self.reader
            .read_exact(&mut buf[PT_HEADER_LEN..total])
            .map_err(map_exact_error)?;
        Ok(total)
    }

    /// Service at most one frame (plus a pending watchdog exit).
    ///
    /// Returns `true` when a request frame was handled. A closed or failing
    /// link triggers the same safety exit as a watchdog expiry; no response is
    /// written because the peer is gone.
    pub fn service(&mut self, now_ms: u32) -> Result<bool, ServerError>
    where
        R: IoRead,
        W: IoWrite,
        B: BusAccess,
    {
        let mut out = [0u8; MAX_FRAME_LEN];
        if let Some(len) = self.poll(now_ms, &mut out)? {
            self.writer
                .write_all(&out[..len])
                .map_err(|e| ServerError::Io(e.kind()))?;
        }

        let mut frame_bytes = [0u8; MAX_FRAME_LEN];
        let len = match self.read_frame(&mut frame_bytes) {
            Ok(len) => len,
            Err(ServerError::UnexpectedEof) | Err(ServerError::Io(_)) => {
                self.exit();
                return Ok(false);
            }
            Err(ServerError::Frame(_)) => {
                // The header is valid enough to echo an opcode; `handle_frame`
                // rejects the oversized length with `InvalidLength`.
                PT_HEADER_LEN
            }
            Err(other) => return Err(other),
        };
        let response_len =
            self.handle_frame(&frame_bytes[..len], now_ms, &mut out)?;
        self.writer
            .write_all(&out[..response_len])
            .map_err(|e| ServerError::Io(e.kind()))?;
        Ok(true)
    }
}

fn requires_passthrough(opcode: u8) -> bool {
    matches!(
        opcode,
        OP_EXIT
            | OP_READ
            | OP_WRITE
            | OP_SYNC_WRITE
            | OP_SCAN_BUS
            | OP_KEEPALIVE
            | OP_EMERGENCY_STOP
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use barefoc_master::{
        Frame as HostFrame, BROADCAST_ID, BUS_BROADCAST_ALL, LOCAL_REG_MODE_STATE,
        REG_TORQUE_ENABLE,
    };
    use crate::buses::{BusAccess, LocalControlTable};

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    struct TestIoError;

    impl IoError for TestIoError {
        fn kind(&self) -> IoErrorKind {
            IoErrorKind::Other
        }
    }

    /// In-memory `embedded_io` stream with independently sized buffers.
    struct TestStream<const N: usize> {
        rx: [u8; N],
        rx_len: usize,
        rx_pos: usize,
        tx: [u8; N],
        tx_len: usize,
    }

    impl<const N: usize> TestStream<N> {
        fn new(rx: &[u8]) -> Self {
            assert!(rx.len() <= N);
            let mut stream = Self {
                rx: [0; N],
                rx_len: rx.len(),
                rx_pos: 0,
                tx: [0; N],
                tx_len: 0,
            };
            stream.rx[..rx.len()].copy_from_slice(rx);
            stream
        }
    }

    impl<const N: usize> embedded_io::ErrorType for TestStream<N> {
        type Error = TestIoError;
    }

    impl<const N: usize> IoWrite for TestStream<N> {
        fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
            let n = buf.len().min(self.tx.len() - self.tx_len);
            self.tx[self.tx_len..self.tx_len + n].copy_from_slice(&buf[..n]);
            self.tx_len += n;
            Ok(n)
        }

        fn flush(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }
    }

    impl<const N: usize> IoRead for TestStream<N> {
        fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
            let remaining = self.rx_len - self.rx_pos;
            let n = buf.len().min(remaining);
            buf[..n].copy_from_slice(&self.rx[self.rx_pos..self.rx_pos + n]);
            self.rx_pos += n;
            Ok(n)
        }
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    struct Record {
        bus: u8,
        node: u8,
        reg: u16,
        payload: [u8; 8],
        len: usize,
    }

    #[derive(Debug)]
    struct MockBus {
        armed: bool,
        disarms: usize,
        active: bool,
        local: LocalControlTable,
        scan_mask: [u8; SCAN_MASK_LEN],
        records: [Record; 8],
        record_len: usize,
    }

    impl MockBus {
        fn new() -> Self {
            Self {
                armed: false,
                disarms: 0,
                active: false,
                local: LocalControlTable::new(0x0002_0007, 0x0B),
                scan_mask: [0; SCAN_MASK_LEN],
                records: [Record {
                    bus: 0,
                    node: 0,
                    reg: 0,
                    payload: [0; 8],
                    len: 0,
                }; 8],
                record_len: 0,
            }
        }

        fn record(&mut self, bus: u8, node: u8, reg: u16, bytes: &[u8]) {
            if self.record_len >= self.records.len() {
                return;
            }
            let mut entry = self.records[self.record_len];
            entry.bus = bus;
            entry.node = node;
            entry.reg = reg;
            entry.len = bytes.len().min(8);
            entry.payload[..entry.len].copy_from_slice(&bytes[..entry.len]);
            self.records[self.record_len] = entry;
            self.record_len += 1;
        }
    }

    impl BusAccess for MockBus {
        fn supported_buses(&self) -> u32 {
            BUS_CAP_LOCAL | 0x03
        }

        fn firmware_version(&self) -> u32 {
            0x0002_0007
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
            self.record(bus, node, reg, bytes);
            Ok(())
        }

        fn read(&mut self, bus: u8, node: u8, reg: u16, out: &mut [u8]) -> Result<(), StatusCode> {
            self.record(bus, node, reg, &[]);
            if bus == BUS_LOCAL && reg == LOCAL_REG_MODE_STATE {
                if out.is_empty() {
                    return Err(StatusCode::InvalidLength);
                }
                out[0] = self.local.mode_state();
            } else {
                for byte in out.iter_mut() {
                    *byte = 0x5A;
                }
            }
            Ok(())
        }

        fn send_packet(&mut self, bus: u8, packet: &[u8]) -> Result<(), StatusCode> {
            self.record(bus, BROADCAST_ID, 0xFFFF, packet);
            Ok(())
        }

        fn scan_i2c(&mut self, _bus: u8, mask: &mut [u8; SCAN_MASK_LEN]) -> Result<(), StatusCode> {
            mask.copy_from_slice(&self.scan_mask);
            Ok(())
        }

        fn broadcast_disarm(&mut self) -> Result<(), StatusCode> {
            self.disarms += 1;
            self.armed = false;
            Ok(())
        }

        fn all_disarmed(&mut self) -> bool {
            !self.armed
        }

        fn set_passthrough_active(&mut self, active: bool) {
            self.active = active;
            self.local.set_mode_state(active);
        }
    }

    type Server = PassthroughServer<TestStream<256>, TestStream<256>, MockBus>;

    fn request(
        opcode: u8,
        bus: u8,
        node: u8,
        reg: u16,
        payload: &[u8],
    ) -> ([u8; MAX_FRAME_LEN], usize) {
        let mut bytes = [0u8; MAX_FRAME_LEN];
        let len = encode_frame(
            &HostFrame::request(opcode, bus, node, reg, payload),
            &mut bytes,
        )
        .unwrap();
        (bytes, len)
    }

    fn status_of(bytes: &[u8]) -> StatusCode {
        decode_frame(bytes).unwrap().status()
    }

    fn enter(server: &mut Server, now_ms: u32) -> StatusCode {
        let (bytes, len) = request(OP_ENTER, BUS_BROADCAST_ALL, BROADCAST_ID, 0, &[]);
        let mut out = [0u8; MAX_FRAME_LEN];
        let n = server.handle_frame(&bytes[..len], now_ms, &mut out).unwrap();
        status_of(&out[..n])
    }

    #[test]
    fn corrupted_frame_is_rejected_without_leaving_mode() {
        let mut server = Server::new(
            TestStream::new(&[]),
            TestStream::new(&[]),
            MockBus::new(),
        );
        assert_eq!(enter(&mut server, 0), StatusCode::Ok);
        let disarms = server.buses().disarms;
        let records = server.buses().record_len;

        let (mut bytes, len) = request(OP_READ, 0, 0x40, 0x144, &[]);
        bytes[len - 1] ^= 0xFF;
        let mut out = [0u8; MAX_FRAME_LEN];
        let n = server.handle_frame(&bytes[..len], 10, &mut out).unwrap();
        assert_eq!(status_of(&out[..n]), StatusCode::InvalidCrc);
        assert!(server.is_active());
        assert_eq!(server.buses().disarms, disarms);
        assert_eq!(server.buses().record_len, records);
    }

    #[test]
    fn entry_is_refused_while_armed() {
        let mut bus = MockBus::new();
        bus.armed = true;
        let mut server = Server::new(TestStream::new(&[]), TestStream::new(&[]), bus);
        assert_eq!(enter(&mut server, 0), StatusCode::NotDisarmed);
        assert!(!server.is_active());
        assert_eq!(server.buses().disarms, 0);
        assert_eq!(server.buses().record_len, 0);
    }

    #[test]
    fn entry_broadcasts_disarm_and_sets_local_mode() {
        let mut server = Server::new(
            TestStream::new(&[]),
            TestStream::new(&[]),
            MockBus::new(),
        );
        assert_eq!(enter(&mut server, 0), StatusCode::Ok);
        assert!(server.is_active());
        assert!(server.buses().active);
        assert_eq!(server.buses().disarms, 1);
        assert_eq!(server.buses().local.mode_state(), 1);
    }

    #[test]
    fn register_frames_require_passthrough() {
        let mut server = Server::new(
            TestStream::new(&[]),
            TestStream::new(&[]),
            MockBus::new(),
        );
        let (bytes, len) = request(OP_READ, 0, 0x40, 0x144, &[0u8; 4]);
        let mut out = [0u8; MAX_FRAME_LEN];
        let n = server.handle_frame(&bytes[..len], 0, &mut out).unwrap();
        assert_eq!(status_of(&out[..n]), StatusCode::NotInPassthrough);
        assert_eq!(server.buses().record_len, 0);
    }

    #[test]
    fn reads_writes_and_raw_packets_are_forwarded() {
        let mut server = Server::new(
            TestStream::new(&[]),
            TestStream::new(&[]),
            MockBus::new(),
        );
        enter(&mut server, 0);

        let (bytes, len) = request(OP_WRITE, 0, 0x40, 0x0130, &1.5f32.to_le_bytes());
        let mut out = [0u8; MAX_FRAME_LEN];
        let n = server.handle_frame(&bytes[..len], 1, &mut out).unwrap();
        assert_eq!(status_of(&out[..n]), StatusCode::Ok);
        let rec = server.buses().records[0];
        assert_eq!((rec.bus, rec.node, rec.reg, rec.len), (0, 0x40, 0x0130, 4));
        assert_eq!(&rec.payload[..4], &1.5f32.to_le_bytes());

        let (bytes, len) = request(OP_READ, barefoc_master::BUS_UART_BASE, 1, 0x144, &[0u8; 4]);
        let n = server.handle_frame(&bytes[..len], 2, &mut out).unwrap();
        let frame = decode_frame(&out[..n]).unwrap();
        assert_eq!(frame.status(), StatusCode::Ok);
        assert_eq!(frame.payload, &[0x5A; 4]);
        assert_eq!(server.buses().records[1].bus, barefoc_master::BUS_UART_BASE);

        let packet = [0xAA, 0xBB, 0xCC];
        let (bytes, len) = request(OP_SYNC_WRITE, 0, 0xFE, REG_TORQUE_ENABLE, &packet);
        let n = server.handle_frame(&bytes[..len], 3, &mut out).unwrap();
        assert_eq!(status_of(&out[..n]), StatusCode::Ok);
        let rec = server.buses().records[2];
        assert_eq!(rec.reg, 0xFFFF);
        assert_eq!(&rec.payload[..3], &packet);
    }

    #[test]
    fn scan_bus_and_get_info_return_single_frame_payloads() {
        let mut buses = MockBus::new();
        buses.scan_mask[8] = 0x03; // addresses 0x40 and 0x41
        buses.scan_mask[15] = 0x80; // address 0x7F
        let mut server = Server::new(TestStream::new(&[]), TestStream::new(&[]), buses);
        enter(&mut server, 0);

        let mut out = [0u8; MAX_FRAME_LEN];
        let (bytes, len) = request(OP_SCAN_BUS, 0, 0, 0, &[]);
        let n = server.handle_frame(&bytes[..len], 1, &mut out).unwrap();
        let frame = decode_frame(&out[..n]).unwrap();
        assert_eq!(frame.opcode, barefoc_master::RESP_SCAN_BUS);
        assert_eq!(frame.payload.len(), SCAN_MASK_LEN);
        assert_eq!(frame.payload[8], 0x03);
        assert_eq!(frame.payload[15], 0x80);

        let (bytes, len) = request(OP_GET_INFO, BUS_LOCAL, 0, 0, &[]);
        let n = server.handle_frame(&bytes[..len], 2, &mut out).unwrap();
        let frame = decode_frame(&out[..n]).unwrap();
        let info = DeviceInfo::decode(frame.payload).unwrap();
        assert_eq!(info.protocol_version, PROTOCOL_VERSION);
        assert_eq!(info.max_payload, PT_MAX_PAYLOAD as u16);
        assert_eq!(info.firmware_version, 0x0002_0007);
        assert!(info.supported_buses & BUS_CAP_LOCAL != 0);
    }

    #[test]
    fn watchdog_expiry_disarms_and_blocks_later_frames() {
        let mut server = Server::new(
            TestStream::new(&[]),
            TestStream::new(&[]),
            MockBus::new(),
        );
        enter(&mut server, 0);
        let disarms = server.buses().disarms;

        let mut out = [0u8; MAX_FRAME_LEN];
        let expired_len = server.poll(500, &mut out).unwrap().unwrap();
        let frame = decode_frame(&out[..expired_len]).unwrap();
        assert_eq!(frame.opcode, RESP_EXIT);
        assert_eq!(frame.status(), StatusCode::Ok);
        assert!(!server.is_active());
        assert!(!server.buses().active);
        assert_eq!(server.buses().local.mode_state(), 0);
        assert_eq!(server.buses().disarms, disarms + 1);

        let (bytes, len) = request(OP_READ, 0, 0x40, 0x144, &[0u8; 4]);
        let n = server.handle_frame(&bytes[..len], 501, &mut out).unwrap();
        assert_eq!(status_of(&out[..n]), StatusCode::NotInPassthrough);
    }

    #[test]
    fn keepalive_refreshes_watchdog_and_local_watchdog_is_configurable() {
        let mut server = Server::new(
            TestStream::new(&[]),
            TestStream::new(&[]),
            MockBus::new(),
        );
        enter(&mut server, 0);
        let mut out = [0u8; MAX_FRAME_LEN];

        let (bytes, len) = request(OP_KEEPALIVE, 0, 0, 0, &[]);
        server.handle_frame(&bytes[..len], 400, &mut out).unwrap();
        assert!(server.poll(499, &mut out).unwrap().is_none());
        assert!(server.poll(900, &mut out).unwrap().is_some());

        let mut server = Server::new(
            TestStream::new(&[]),
            TestStream::new(&[]),
            MockBus::new(),
        );
        enter(&mut server, 0);
        let (bytes, len) = request(
            OP_WRITE,
            BUS_LOCAL,
            0,
            LOCAL_REG_WATCHDOG_TIMEOUT_MS,
            &1000u16.to_le_bytes(),
        );
        server.handle_frame(&bytes[..len], 0, &mut out).unwrap();
        assert_eq!(server.watchdog_ms(), 1000);
    }

    #[test]
    fn emergency_stop_exits_immediately() {
        let mut server = Server::new(
            TestStream::new(&[]),
            TestStream::new(&[]),
            MockBus::new(),
        );
        enter(&mut server, 0);
        let mut out = [0u8; MAX_FRAME_LEN];
        let (bytes, len) = request(OP_EMERGENCY_STOP, BUS_BROADCAST_ALL, 0xFE, 0, &[]);
        let n = server.handle_frame(&bytes[..len], 1, &mut out).unwrap();
        assert_eq!(status_of(&out[..n]), StatusCode::Ok);
        assert!(!server.is_active());
        assert_eq!(server.buses().disarms, 2);
    }

    #[test]
    fn equivalent_streams_produce_byte_identical_responses() {
        let (bytes, len) = request(OP_GET_INFO, BUS_LOCAL, 0, 0, &[]);

        let mut a = PassthroughServer::new(
            TestStream::<256>::new(&bytes[..len]),
            TestStream::<256>::new(&[]),
            MockBus::new(),
        );
        let mut b = PassthroughServer::new(
            TestStream::<512>::new(&bytes[..len]),
            TestStream::<512>::new(&[]),
            MockBus::new(),
        );
        // GET_INFO is legal in normal mode; service() reads and answers it.
        assert!(a.service(0).unwrap());
        assert!(b.service(0).unwrap());
        let (_, stream_a, _) = a.into_parts();
        let (_, stream_b, _) = b.into_parts();
        assert_eq!(stream_a.tx[..stream_a.tx_len], stream_b.tx[..stream_b.tx_len]);
    }

    #[test]
    fn link_loss_exits_without_response() {
        let mut server = Server::new(
            TestStream::new(&[]),
            TestStream::new(&[]),
            MockBus::new(),
        );
        enter(&mut server, 0);
        let disarms = server.buses().disarms;
        assert!(!server.service(1).unwrap());
        assert!(!server.is_active());
        assert_eq!(server.buses().disarms, disarms + 1);
    }
}
