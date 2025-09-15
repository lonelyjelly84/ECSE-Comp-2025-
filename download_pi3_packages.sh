#!/bin/bash
# Download script for Pi 3 offline packages
# Run this on an INTERNET-CONNECTED machine to download packages for offline Pi 3
# Optimized for Python 3.9 on 64-bit Pi OS

echo "=============================================="
echo "Pi 3 Offline Package Downloader (Python 3.9)"
echo "=============================================="
echo "This script downloads packages for offline Pi 3 installation"
echo "Target: Pi 3 64-bit OS with Python 3.9"
echo "Run this on a machine WITH internet access"
echo ""

# Verify Python version on download machine
echo "Download machine Python version: $(python3 --version)"
echo ""

# Create directories
mkdir -p rpi3_packages
cd rpi3_packages

echo "Downloading Python 3.9 compatible packages for Pi 3 (aarch64)..."

# Download packages with specific Python 3.9 and Pi 3 compatible versions
echo "1. Downloading setuptools..."
pip3 download --platform linux_aarch64 --python-version 39 --only-binary=:all: --no-deps setuptools==68.2.2

echo "2. Downloading wheel..."
pip3 download --platform linux_aarch64 --python-version 39 --only-binary=:all: --no-deps wheel==0.41.2

echo "3. Downloading numpy (Python 3.9 + Pi 3 optimized)..."
pip3 download --platform linux_aarch64 --python-version 39 --only-binary=:all: --no-deps "numpy>=1.19.5,<1.25.0"

echo "4. Downloading OpenCV (headless, Python 3.9 compatible)..."
pip3 download --platform linux_aarch64 --python-version 39 --only-binary=:all: --no-deps opencv-python-headless==4.6.0.66

echo "5. Downloading Pillow (Python 3.9 compatible)..."
pip3 download --platform linux_aarch64 --python-version 39 --only-binary=:all: --no-deps "Pillow>=8.3.0,<11.0.0"

echo "6. Downloading pygame (Python 3.9 compatible)..."
pip3 download --platform linux_aarch64 --python-version 39 --only-binary=:all: --no-deps pygame==2.1.2

echo "7. Downloading smbus2 (pure Python - no architecture dependency)..."
pip3 download --no-deps smbus2==0.4.2

echo "8. Downloading gpiozero dependencies..."
pip3 download --no-deps colorzero==1.1
pip3 download --no-deps gpiozero==1.6.2

echo "9. Downloading additional dependencies for Python 3.9..."
pip3 download --no-deps six==1.16.0

echo ""
echo "=============================================="
echo "Download complete!"
echo "=============================================="

# List downloaded files
echo "Downloaded packages:"
ls -la *.whl

echo ""
echo "Total package size:"
du -sh .

echo ""
echo "=============================================="
echo "TRANSFER INSTRUCTIONS:"
echo "=============================================="
echo "1. Copy the entire 'rpi3_packages' folder to USB drive"
echo "2. Transfer USB drive to your offline Pi 3"
echo "3. Copy rpi3_packages folder to your ECSE-Comp-2025- directory"
echo "4. Run the offline installation: ./install_pi3_offline.sh"
echo ""
echo "Alternative: Copy to SD card with Pi 3 OS before first boot"
echo "=============================================="