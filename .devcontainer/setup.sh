#!/bin/bash
set -e

echo "Setting up Fevicol development environment..."

# Install system dependencies required by probe-rs-tools
echo "Installing system dependencies..."
sudo apt-get update && sudo apt-get install -y libudev-dev

# Install probe-rs tools for flashing and debugging
echo "Installing probe-rs-tools..."
cargo install probe-rs-tools --locked

# Install useful development tools
echo "Installing additional tools..."
cargo install cargo-expand || true

# Set up udev rules for probe-rs (only if /dev/bus/usb exists - i.e., local dev with USB passthrough)
if [ -d "/dev/bus/usb" ]; then
    echo "Configuring USB permissions..."
    cat > /tmp/99-probe-rs.rules << 'EOF'
# probe-rs rules
SUBSYSTEM=="usb", ATTR{idVendor}=="303a", MODE="0666"
SUBSYSTEM=="usb", ATTR{idVendor}=="10c4", MODE="0666"
EOF
    sudo mv /tmp/99-probe-rs.rules /etc/udev/rules.d/ || true
    sudo udevadm control --reload-rules || true
    sudo udevadm trigger || true
else
    echo "USB devices not available (CI environment). Skipping udev configuration."
fi

echo "Development environment setup complete!"
echo ""
echo "You can now:"
echo "  - Build the project: cargo build"
echo "  - Run tests: cargo test"
echo "  - Flash to device: cargo run (requires connected ESP32-C6)"
echo ""
