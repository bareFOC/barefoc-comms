//! # `barefoc-passthrough` — device-side bus-passthrough server
//!
//! A `#![no_std]`, zero-allocation server that turns framed PC ↔ flight-controller
//! requests into ordinary [`barefoc_master::Transport`] calls on downstream
//! I2C / framed-UART buses or on the flight controller's own control table.
//!
//! ## Safety model
//!
//! ```text
//! NORMAL --OP_ENTER(disarmed only)--> PASSTHROUGH
//!    ^                                    |
//!    +-- OP_EXIT / watchdog / link loss / fault --+
//! ```
//!
//! `OP_ENTER` is refused with `NotDisarmed` while the FC or any downstream
//! node reports torque enabled. On entry and on every exit path the server
//! broadcasts `REG_TORQUE_ENABLE = 0`, flips the mode-state register on bus
//! `0xF0`, and calls [`BusAccess::set_passthrough_active`] so the application
//! restores its flight-control bus roles. A malformed frame only rejects that
//! frame; it never exits passthrough.
//!
//! The server is generic over `embedded_io::Read + Write`, so native USB CDC,
//! USART CDC and RTT attachments produce identical responses. Frame codec and
//! constants are shared with the host through `barefoc-master`.
//!
//! Upstream: `barefoc-comms/barefoc-multibus/src/transport.rs`
//! (`TargetTransport` concurrency contract), `barefoc-master/src/transport.rs`
//! (inner transport adapters), `openspec/changes/bus-passthrough/design.md`.
//! There is no C++ equivalent.

#![no_std]

pub mod buses;
pub mod server;

pub use buses::{
    BusAccess, BusRouter, DeviceBuses, LocalControlTable,
};
pub use server::{
    Mode, PassthroughServer, ServerError, DEFAULT_WATCHDOG_MS,
};
