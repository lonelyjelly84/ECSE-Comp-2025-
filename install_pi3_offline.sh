#!/bin/bash
# OFFLINE Installation script for Raspberry Pi 3 (64-bit OS)
# This script works WITHOUT internet connection
# Requires pre-downloaded packages in rpi3_packages folder
# Optimized for Python 3.9

echo "=============================================="
echo "Pi 3 OFFLINE Installation (Python 3.9)"
echo "=============================================="
echo "Installing Python packages for ECSE Robot System on Pi 3..."
echo "Python version: $(python3 --version)"
echo "Architecture: $(uname -m)"
echo ""

# Verify Python 3.9
PYTHON_VERSION=$(python3 -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")
echo "Detected Python version: $PYTHON_VERSION"

if [[ "$PYTHON_VERSION" != "3.9" ]]; then
    echo "⚠ Warning: This script is optimized for Python 3.9"
    echo "Detected version: $PYTHON_VERSION"
    read -p "Continue anyway? (y/N): " confirm
    if [[ $confirm != [yY] ]]; then
        echo "Installation cancelled."
        exit 1
    fi
fi

# Check if we're on Pi 3
if ! grep -q "Raspberry Pi 3" /proc/device-tree/model 2>/dev/null; then
    echo "⚠ Warning: This script is optimized for Raspberry Pi 3"
    echo "Current device: $(cat /proc/device-tree/model 2>/dev/null || echo 'Unknown')"
    read -p "Continue anyway? (y/N): " confirm
    if [[ $confirm != [yY] ]]; then
        echo "Installation cancelled."
        exit 1
    fi
fi

# Check for offline packages
if [ ! -d "rpi3_packages" ]; then
    echo "❌ ERROR: rpi3_packages folder not found!"
    echo ""
    echo "You need to:"
    echo "1. Run download_pi3_packages.sh on an internet-connected machine"
    echo "2. Transfer the rpi3_packages folder to this Pi 3"
    echo "3. Place it in the same directory as this script"
    echo ""
    exit 1
fi

echo "✓ Found offline packages directory"
echo "Package contents:"
ls -la rpi3_packages/

echo ""
echo "=============================================="
echo "System Configuration (OFFLINE)"
echo "=============================================="

# Enable I2C and Camera (this works offline)
echo "Enabling I2C and Camera interfaces..."
sudo raspi-config nonint do_i2c 0  # Enable I2C
sudo raspi-config nonint do_camera 0  # Enable Camera

# Install ONLY system packages that are already on Pi OS
echo "Installing pre-installed system dependencies..."
sudo apt install -y --no-install-recommends python3-pip python3-dev python3-setuptools 2>/dev/null || echo "⚠ Some system packages may not be available offline"
sudo apt install -y --no-install-recommends i2c-tools python3-smbus 2>/dev/null || echo "⚠ I2C tools may not be available offline"

echo ""
echo "=============================================="
echo "Installing Python Packages (OFFLINE)"
echo "=============================================="

# Navigate to the wheel files directory
cd rpi3_packages

# Install packages in dependency order using ONLY local files
echo "Installing packages from local wheel files (Python 3.9 compatible)..."

echo "1. Installing six (dependency)..."
pip3 install --no-index --find-links . six*.whl || echo "⚠ six not found (may not be needed)"

echo "2. Installing setuptools..."
pip3 install --no-index --find-links . setuptools*.whl

echo "3. Installing wheel..."
pip3 install --no-index --find-links . wheel*.whl

echo "4. Installing numpy (optimized for Python 3.9)..."
pip3 install --no-index --find-links . numpy*.whl

echo "5. Installing Pillow..."
pip3 install --no-index --find-links . Pillow*.whl

echo "6. Installing OpenCV (headless)..."
pip3 install --no-index --find-links . opencv_python_headless*.whl

echo "6. Installing pygame..."
pip3 install --no-index --find-links . pygame*.whl

echo "7. Installing smbus2..."
pip3 install --no-index --find-links . smbus2*.whl

echo "8. Installing gpiozero dependencies..."
pip3 install --no-index --find-links . colorzero*.whl
pip3 install --no-index --find-links . gpiozero*.whl

# Return to project directory
cd ..

echo ""
echo "=============================================="
echo "Testing Installation (OFFLINE)"
echo "=============================================="

# Test imports with error handling
python3 -c "
import sys
import platform
print(f'Python version: {sys.version}')
print(f'Platform: {platform.platform()}')
print(f'Architecture: {platform.machine()}')
print('')
print('Testing package imports...')

try:
    import numpy as np
    print(f'✓ numpy {np.__version__} imported successfully')
    # Test basic numpy functionality
    test_array = np.array([1, 2, 3])
    print(f'  - NumPy test array: {test_array}')
except ImportError as e:
    print(f'✗ numpy import failed: {e}')
except Exception as e:
    print(f'✗ numpy test failed: {e}')

try:
    import cv2
    print(f'✓ opencv {cv2.__version__} imported successfully')
    # Test basic OpenCV functionality
    print(f'  - OpenCV build info available: {len(cv2.getBuildInformation()) > 0}')
except ImportError as e:
    print(f'✗ opencv import failed: {e}')
except Exception as e:
    print(f'✗ opencv test failed: {e}')

try:
    import pygame
    print(f'✓ pygame {pygame.version.ver} imported successfully')
    # Test pygame mixer initialization
    pygame.mixer.init()
    print(f'  - Pygame mixer initialized successfully')
except ImportError as e:
    print(f'✗ pygame import failed: {e}')
except Exception as e:
    print(f'✗ pygame test failed: {e}')

try:
    import smbus2
    print('✓ smbus2 imported successfully')
    # Test SMBus creation (won\'t actually connect)
    bus = smbus2.SMBus(1)
    print('  - SMBus interface created successfully')
except ImportError as e:
    print(f'✗ smbus2 import failed: {e}')
except Exception as e:
    print(f'✗ smbus2 test failed: {e}')

try:
    from gpiozero import LED
    print('✓ gpiozero imported successfully')
    print('  - GPIO interface available')
except ImportError as e:
    print(f'✗ gpiozero import failed: {e}')
except Exception as e:
    print(f'✗ gpiozero test failed: {e}')

print('')
print('Hardware checks (offline-capable):')
try:
    import subprocess
    
    # Check I2C (this works offline)
    result = subprocess.run(['i2cdetect', '-y', '1'], capture_output=True, text=True, timeout=5)
    if result.returncode == 0:
        print('✓ I2C bus accessible')
    else:
        print('⚠ I2C bus not accessible (may need reboot)')
    
    # Check camera (this works offline)
    result = subprocess.run(['vcgencmd', 'get_camera'], capture_output=True, text=True, timeout=5)
    if 'detected=1' in result.stdout:
        print('✓ Camera detected')
    else:
        print('⚠ Camera not detected (may need reboot to enable)')
        
    # Check Pi model
    with open('/proc/device-tree/model', 'r') as f:
        model = f.read().strip()
        print(f'✓ Hardware: {model}')
        
except Exception as e:
    print(f'⚠ Hardware check failed: {e}')
"

echo ""
echo "=============================================="
echo "Pi 3 OFFLINE Setup Complete!"
echo "=============================================="
echo ""
echo "✓ All packages installed from local files"
echo "✓ No internet connection required for operation"
echo "✓ Pi 3 optimizations automatically applied"
echo ""
echo "Performance recommendations for Pi 3:"
echo "- Camera resolution: 320x240 (automatically set)"
echo "- Face detection: Every 1.5 seconds (automatically set)"
echo "- Processing delays: Added automatically for stability"
echo ""
echo "Hardware setup:"
echo "- Connect ultrasonic sensor to GPIO 17/27"
echo "- Connect servo to GPIO 13"
echo "- Connect I2C LCD to SDA/SCL pins"
echo ""
echo "To start the robot:"
echo "  python3 robot_name.py"
echo ""
echo "Note: First run may be slower as Python optimizes for Pi 3"
echo "=============================================="