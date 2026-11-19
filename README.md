# barefoc-comms

Communication protocols, serial command processors, and bus transports for the [BareFOC](https://github.com/bareFOC/bareFOC) ecosystem.

This workspace provides `#![no_std]` streaming and memory-mapped communication interfaces for motion control telemetry, live tuning, and bus control over UART, RTT, I2C, SPI, and USB.

---

## Workspace Structure

```
barefoc-comms/
├── barefoc-commander/   # ASCII command line parser implementing SimpleFOC commands.h
├── barefoc-fibre/       # Fibre RPC protocol server (compatible with ODrive Web GUI)
├── barefoc-multibus/    # Memory-mapped register-table server/client framework
└── barefoc-rtt/         # SEGGER RTT byte-stream transport implementing embedded-io
```

---

## Crates Overview

### 1. `barefoc-commander` (ASCII Command Interface)
A pure-Rust, `#![no_std]` implementation of the SimpleFOC Commander protocol:
- Parses the single-character command grammar defined in `commands.h` (`T` for target, `M` for motion mode, `C` for current, `V` for velocity, etc.).
- Generic streaming `LineProcessor` operating over `embedded_io::Read` and `embedded_io::Write`.
- Dispatches commands directly to motor configurations and register targets with configurable verbosity (`Nothing`, `Minimal`, `UserFriendly`, `MachineReadable`).

### 2. `barefoc-fibre` (Binary RPC Interface)
Zero-allocation implementation of the Fibre RPC protocol:
- Compatible with the official [ODrive Web GUI](https://gui.odriverobotics.com) and `odrivetool`.
- Handles packet framing, CRC-8 / CRC-16 checks, and JSON Endpoint 0 schema negotiation.
- Enables browser-based live plotting, parameter configuration, and motor control over serial, TCP, or USB CDC.

### 3. `barefoc-multibus` (Register-Mapped Bus Protocol)
Descriptor-driven control table framework for address-based buses (I2C, SPI, I3C):
- Memory-mapped server core (`RegMap`) with register descriptors (offset, width, access permissions, and write hooks).
- Supports register-pointer latching and auto-increment burst reads/writes.
- Self-clearing command registers for action triggers (e.g., zeroing, calibration, soft-reset).
- Includes controller-side client abstractions (`RegClientI2c`, `RegClientSpi`) for host-side round-trip testing.

### 4. `barefoc-rtt` (SEGGER Real-Time Transfer Transport)
Zero-allocation RTT byte stream implementing the `embedded-io` traits:
- Adapts SEGGER RTT up/down channels into an `RttStream<Up, Down>` compatible with `embedded_io::Read` and `embedded_io::Write`.
- Supports non-blocking operations for integration with serial processors like `barefoc-commander`.
- Application owns RTT initialization while the transport manages channel streaming.

---

## Usage Example

### Streaming Console with Commander over RTT or UART:

```rust,no_run
use barefoc_commander::{Commander, LineProcessor};
use barefoc_commander::motor::MotorCommander;
use embedded_io::{Read, Write};

fn process_console<IO: Read + Write>(io: &mut IO, commander: &mut Commander) {
    let mut processor = LineProcessor::<64>::new();
    let mut buffer = [0u8; 16];

    if let Ok(n) = io.read(&mut buffer) {
        if n > 0 {
            let _ = processor.process_slice(&buffer[..n], |line| {
                commander.run(line);
            });
        }
    }
}
```

---

## Testing

Host unit tests verify protocol parsing, CRC integrity, and framing:

```bash
cargo test -p barefoc-commander
cargo test -p barefoc-fibre
cargo test -p barefoc-multibus
cargo test -p barefoc-rtt
```

---

## License

This project is licensed under the MIT License. See individual crate directories for license files.
