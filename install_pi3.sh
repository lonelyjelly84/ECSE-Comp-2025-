#!/bin/bash
# Installation script for Raspberry Pi 3 (64-bit OS)
# Automatically detects online/offline mode

echo "=============================================="
echo "Pi 3 Installation Script"
echo "=============================================="
echo "Python version: $(python3 --version)"
echo "Architecture: $(uname -m)"

# Check if we're on Pi 3
if ! grep -q "Raspberry Pi 3" /proc/device-tree/model 2>/dev/null; then
    echo "Warning: This script is optimized for Raspberry Pi 3"
    echo "Current device: $(cat /proc/device-tree/model 2>/dev/null || echo 'Unknown')"
    read -p "Continue anyway? (y/N): " confirm
    if [[ $confirm != [yY] ]]; then
        echo "Installation cancelled."
        exit 1
    fi
fi

# Check for internet connectivity
echo "Checking internet connectivity..."
if ping -c 1 8.8.8.8 &> /dev/null; then
    echo "✓ Internet connection detected - using ONLINE installation"
    ONLINE_MODE=true
else
    echo "✗ No internet connection - checking for offline packages"
    ONLINE_MODE=false
    
    if [ -f "install_pi3_offline.sh" ] && [ -d "rpi3_packages" ]; then
        echo "✓ Found offline installation files"
        echo "Switching to OFFLINE installation mode..."
        exec ./install_pi3_offline.sh
    else
        echo ""
        echo "❌ ERROR: No internet and no offline packages found!"
        echo ""
        echo "For OFFLINE installation, you need:"
        echo "1. Run 'download_pi3_packages.sh' on an internet-connected machine"
        echo "2. Transfer the 'rpi3_packages' folder to this Pi 3"
        echo "3. Ensure 'install_pi3_offline.sh' is present"
        echo ""
        echo "Then run this script again."
        exit 1
    fi
fi

echo "Proceeding with ONLINE installation..."

# Enable I2C and Camera (required for robot features)
echo "Enabling I2C and Camera interfaces..."
sudo raspi-config nonint do_i2c 0  # Enable I2C
sudo raspi-config nonint do_camera 0  # Enable Camera

# Update package list and install system dependencies
echo "Installing system dependencies..."
sudo apt update
sudo apt install -y python3-pip python3-dev python3-setuptools
sudo apt install -y libatlas-base-dev libhdf5-dev libhdf5-serial-dev
sudo apt install -y libjasper-dev libqtgui4 libqt4-test
sudo apt install -y i2c-tools python3-smbus

# Install packages using pip (let Pi 3 compile optimized versions)
echo "Installing Python packages (this may take a while on Pi 3)..."

# Install basic packages first
pip3 install --upgrade pip setuptools wheel

# Install numpy (optimized for Pi 3)
echo "Installing numpy (optimized build for Pi 3)..."
pip3 install numpy==1.21.6  # Stable version for Pi 3

# Install OpenCV with reduced functionality for better Pi 3 performance
echo "Installing OpenCV (headless version for better performance)..."
pip3 install opencv-python-headless==4.6.0.66

# Install other packages
echo "Installing additional packages..."
pip3 install smbus2==0.4.2
pip3 install pygame==2.1.2
pip3 install pillow==9.5.0

# Install GPIO library
echo "Installing GPIO control library..."
pip3 install gpiozero

echo ""
echo "Testing package imports..."

# Test imports with error handling
python3 -c "
import sys
print(f'Python version: {sys.version}')
print(f'Platform: {sys.platform}')
print('Testing imports...')

try:
    import numpy as np
    print(f'✓ numpy {np.__version__} imported successfully')
    print(f'  Compiled with: {np.show_config()}' if hasattr(np, 'show_config') else '')
except ImportError as e:
    print(f'✗ numpy import failed: {e}')

try:
    import cv2
    print(f'✓ opencv {cv2.__version__} imported successfully')
    print(f'  Build info: {cv2.getBuildInformation()[:100]}...')
except ImportError as e:
    print(f'✗ opencv import failed: {e}')

try:
    import pygame
    print(f'✓ pygame {pygame.version.ver} imported successfully')
except ImportError as e:
    print(f'✗ pygame import failed: {e}')

try:
    import smbus2
    print('✓ smbus2 imported successfully')
except ImportError as e:
    print(f'✗ smbus2 import failed: {e}')

try:
    from gpiozero import LED
    print('✓ gpiozero imported successfully')
except ImportError as e:
    print(f'✗ gpiozero import failed: {e}')

print('')
print('Checking hardware capabilities...')
try:
    import subprocess
    
    # Check I2C
    result = subprocess.run(['i2cdetect', '-y', '1'], capture_output=True, text=True)
    if result.returncode == 0:
        print('✓ I2C bus accessible')
    else:
        print('✗ I2C bus not accessible')
    
    # Check camera
    result = subprocess.run(['vcgencmd', 'get_camera'], capture_output=True, text=True)
    if 'detected=1' in result.stdout:
        print('✓ Camera detected')
    else:
        print('⚠ Camera not detected or not enabled')
        
except Exception as e:
    print(f'Hardware check failed: {e}')
"

echo ""
echo "Pi 3 setup complete!"
echo ""
echo "Performance recommendations for Pi 3:"
echo "- Use lower camera resolution (320x240)"
echo "- Reduce face detection frequency"
echo "- Consider overclocking (sudo raspi-config -> Advanced Options -> Overclock)"
echo ""
echo "To start the robot: python3 Software/integrated_robot_system.py"
echo "Note: First run may be slower as Python optimizes bytecode for Pi 3"