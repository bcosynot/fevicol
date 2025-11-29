# Development Container Configuration

This directory contains the devcontainer configuration for the Fevicol ESP32-C6 project.

## What's Included

The devcontainer provides a complete development environment with:

- **Rust toolchain** (stable) with RISC-V target support
- **probe-rs-tools** for flashing and debugging ESP32-C6
- **rust-src** component for `no_std` bare-metal builds
- **VS Code extensions** for Rust development
- **USB passthrough** for connecting to physical hardware

## Quick Start

### Using VS Code

1. Install the "Dev Containers" extension in VS Code
2. Open the project folder
3. Click "Reopen in Container" when prompted (or use Command Palette → "Dev Containers: Reopen in Container")
4. Wait for the container to build and setup to complete

### Using GitHub Codespaces

1. Click "Code" → "Codespaces" → "Create codespace on \<branch\>"
2. Wait for the environment to initialize
3. The project will be ready to build and test

## Building and Testing

Once inside the container:

```bash
# Build the project
cargo build

# Run tests
cargo test

# Check formatting
cargo fmt --all -- --check

# Run linter
cargo clippy --all-features --workspace -- -D warnings
```

## Flashing to Hardware

To flash firmware to a connected ESP32-C6 device:

1. Connect the device via USB
2. The container has USB passthrough enabled with `--privileged` mode
3. Run: `cargo run`

Note: USB passthrough works best with Docker Desktop or local VS Code with Dev Containers. It may have limitations in browser-based Codespaces.

## Configuration Details

- **Base Image**: Microsoft Rust devcontainer (Debian Bookworm)
- **Post-Create Setup**: Automatically installs probe-rs and configures Rust target
- **USB Access**: Configured for ESP32 USB devices (vendor IDs: 303a, 10c4)
- **VS Code Settings**: Pre-configured rust-analyzer for RISC-V embedded development

## Troubleshooting

### probe-rs not found
The setup script installs probe-rs automatically. If missing, run:
```bash
cargo install probe-rs-tools --locked
```

### USB device not accessible
Check that:
- The container is running with `--privileged` flag
- USB device is connected and visible: `lsusb`
- udev rules are loaded: `sudo udevadm control --reload-rules`

### Build errors
Ensure all components are installed:
```bash
rustup target add riscv32imac-unknown-none-elf
rustup component add rust-src
```
