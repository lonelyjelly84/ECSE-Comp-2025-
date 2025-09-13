#!/bin/bash
# Installation script for Raspberry Pi 4 (64-bit OS)
# Run this script on your offline Pi 4 to install dependencies

echo "Installing Python packages for ECSE Robot System on Pi 4..."
echo "Python version: $(python3 --version)"

# Navigate to the wheel files directory
cd "$(dirname "$0")/rpi_packages"

# Install packages in dependency order
echo "Installing setuptools..."
pip3 install --no-index --find-links . setuptools-80.9.0-py3-none-any.whl

echo "Installing wheel..."
pip3 install --no-index --find-links . wheel-0.45.1-py3-none-any.whl

echo "Installing numpy..."
pip3 install --no-index --find-links . numpy-1.24.4-cp39-cp39-manylinux_2_17_aarch64.manylinux2014_aarch64.whl

echo "Installing Pillow..."
pip3 install --no-index --find-links . pillow-11.3.0-cp39-cp39-manylinux2014_aarch64.manylinux_2_17_aarch64.whl

echo "Installing OpenCV..."
pip3 install --no-index --find-links . opencv_python-4.12.0.88-cp37-abi3-manylinux2014_aarch64.manylinux_2_17_aarch64.whl

echo "Installing pygame..."
pip3 install --no-index --find-links . pygame-2.6.1-cp39-cp39-manylinux_2_17_aarch64.manylinux2014_aarch64.whl

echo "Installing smbus2..."
pip3 install --no-index --find-links . smbus2-0.5.0-py2.py3-none-any.whl

echo "Installing lgpio..."
pip3 install --no-index --find-links . lgpio-0.2.2.0-cp39-cp39-manylinux_2_34_aarch64.whl

echo ""
echo "Installation complete! Testing imports..."

# Test imports
python3 -c "
import sys
print(f'Python version: {sys.version}')
print('Testing imports...')

try:
    import numpy as np
    print('✓ numpy imported successfully')
except ImportError as e:
    print(f'✗ numpy import failed: {e}')

try:
    import cv2
    print('✓ opencv imported successfully')
except ImportError as e:
    print(f'✗ opencv import failed: {e}')

try:
    import pygame
    print('✓ pygame imported successfully')
except ImportError as e:
    print(f'✗ pygame import failed: {e}')

try:
    import smbus2
    print('✓ smbus2 imported successfully')
except ImportError as e:
    print(f'✗ smbus2 import failed: {e}')

try:
    import lgpio
    print('✓ lgpio imported successfully')
    print(f'lgpio version: {lgpio.get_library_version()}')
except ImportError as e:
    print(f'✗ lgpio import failed: {e}')
"

echo ""
echo "Pi 4 setup complete! You can now run the robot system."
echo "To start the robot: python3 Software/integrated_robot_system.py"