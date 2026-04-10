#!/bin/bash
#
# AGV Master-Slave Setup Script for Raspberry Pi
# Run this script once to install all dependencies
#

echo "=================================="
echo "AGV Master-Slave Setup for RPi"
echo "=================================="
echo ""

# Update system packages
echo "[1] Updating system packages..."
sudo apt-get update
sudo apt-get upgrade -y

# Install Python dependencies
echo ""
echo "[2] Installing Python dependencies..."
pip3 install --upgrade pip
pip3 install pyserial opencv-python apriltag numpy

# Install git (for apriltag if needed)
echo ""
echo "[3] Installing development tools..."
sudo apt-get install -y git build-essential cmake libopencv-dev

# Check serial port permissions
echo ""
echo "[4] Checking serial port access..."
if groups | grep -q dialout; then
    echo "✓ User has dialout group access"
else
    echo "! Adding user to dialout group for serial access..."
    sudo usermod -a -G dialout $USER
    echo "! Please log out and log back in for group changes to take effect"
fi

# Check if camera is available
echo ""
echo "[5] Checking camera..."
if [ -c /dev/video0 ]; then
    echo "✓ Camera detected at /dev/video0"
else
    echo "✗ Camera not detected - make sure it's connected"
fi

# List available serial ports
echo ""
echo "[6] Available serial ports:"
ls -la /dev/tty* | grep -E "USB|AMA"

echo ""
echo "=================================="
echo "Setup Complete!"
echo "=================================="
echo ""
echo "Next steps:"
echo "1. Upload agv_controller_slave.ino to Arduino/Mega using Arduino IDE"
echo "2. Note the serial port (usually /dev/ttyUSB0 or /dev/ttyACM0)"
echo "3. Update SERIAL_PORT in camera_detection_master.py if needed"
echo "4. Run: python3 camera_detection_master.py"
echo ""
