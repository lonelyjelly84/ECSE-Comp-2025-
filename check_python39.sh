#!/bin/bash
# Python 3.9 Compatibility Checker for Pi 3 Offline Setup
# Run this to verify Python 3.9 setup before package installation

echo "=============================================="
echo "Python 3.9 Compatibility Checker"
echo "=============================================="

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo "Checking Python 3.9 setup on Pi 3..."
echo ""

# Check Python version
echo "=== Python Version Analysis ==="
PYTHON_VERSION=$(python3 --version 2>&1)
echo "Installed Python: $PYTHON_VERSION"

PYTHON_MAJOR=$(python3 -c "import sys; print(sys.version_info.major)" 2>/dev/null)
PYTHON_MINOR=$(python3 -c "import sys; print(sys.version_info.minor)" 2>/dev/null)
PYTHON_MICRO=$(python3 -c "import sys; print(sys.version_info.micro)" 2>/dev/null)

if [[ "$PYTHON_MAJOR" == "3" ]] && [[ "$PYTHON_MINOR" == "9" ]]; then
    echo -e "${GREEN}✓ Python 3.9.${PYTHON_MICRO} detected - COMPATIBLE${NC}"
else
    echo -e "${RED}✗ Python $PYTHON_MAJOR.$PYTHON_MINOR detected - May have compatibility issues${NC}"
    echo -e "${YELLOW}⚠ This setup is optimized for Python 3.9${NC}"
fi

echo ""

# Check Pi model
echo "=== Hardware Compatibility ==="
PI_MODEL=$(cat /proc/device-tree/model 2>/dev/null || echo "Unknown")
echo "Hardware: $PI_MODEL"

if [[ "$PI_MODEL" == *"Raspberry Pi 3"* ]]; then
    echo -e "${GREEN}✓ Pi 3 detected - COMPATIBLE${NC}"
elif [[ "$PI_MODEL" == *"Raspberry Pi 4"* ]]; then
    echo -e "${YELLOW}⚠ Pi 4 detected - Will work but not optimized${NC}"
else
    echo -e "${YELLOW}⚠ Unknown Pi model - May work${NC}"
fi

# Check architecture
ARCH=$(uname -m)
echo "Architecture: $ARCH"

if [[ "$ARCH" == "aarch64" ]]; then
    echo -e "${GREEN}✓ 64-bit ARM detected - COMPATIBLE${NC}"
else
    echo -e "${RED}✗ $ARCH detected - Package compatibility issues likely${NC}"
fi

echo ""

# Check Python capabilities
echo "=== Python 3.9 Capabilities Test ==="

# Test basic imports
python3 -c "
import sys
import os
import platform

print(f'Python executable: {sys.executable}')
print(f'Python path: {sys.path[0]}')
print(f'Platform: {platform.platform()}')
print(f'Machine: {platform.machine()}')

# Test pip availability
try:
    import pip
    print(f'Pip version: {pip.__version__}')
except ImportError:
    print('Pip: Not available')

# Test development headers
try:
    import distutils.util
    print('Development headers: Available')
except ImportError:
    print('Development headers: Not available')

# Test compiled extensions capability
try:
    import ctypes
    print('C extensions: Supported')
except ImportError:
    print('C extensions: Not supported')
"

echo ""

# Check for existing packages that might conflict
echo "=== Existing Package Check ==="
echo "Checking for potentially conflicting packages..."

CONFLICTS=0

if python3 -c "import numpy" 2>/dev/null; then
    NUMPY_VER=$(python3 -c "import numpy; print(numpy.__version__)" 2>/dev/null)
    echo "NumPy already installed: $NUMPY_VER"
    # Check if it's a compatible version
    if python3 -c "import numpy; assert numpy.__version__ >= '1.19.0'" 2>/dev/null; then
        echo -e "${GREEN}✓ Compatible NumPy version${NC}"
    else
        echo -e "${YELLOW}⚠ Old NumPy version - will be upgraded${NC}"
        ((CONFLICTS++))
    fi
else
    echo "NumPy: Not installed"
fi

if python3 -c "import cv2" 2>/dev/null; then
    CV2_VER=$(python3 -c "import cv2; print(cv2.__version__)" 2>/dev/null)
    echo "OpenCV already installed: $CV2_VER"
    echo -e "${YELLOW}⚠ Existing OpenCV - may conflict with headless version${NC}"
    ((CONFLICTS++))
else
    echo "OpenCV: Not installed"
fi

if python3 -c "import pygame" 2>/dev/null; then
    PYGAME_VER=$(python3 -c "import pygame; print(pygame.version.ver)" 2>/dev/null)
    echo "Pygame already installed: $PYGAME_VER"
    echo -e "${GREEN}✓ Pygame present${NC}"
else
    echo "Pygame: Not installed"
fi

echo ""

# Final recommendations
echo "=== Recommendations ==="

if [[ "$PYTHON_MAJOR" == "3" ]] && [[ "$PYTHON_MINOR" == "9" ]] && [[ "$ARCH" == "aarch64" ]]; then
    echo -e "${GREEN}✅ System is READY for offline installation${NC}"
    echo "Your Python 3.9 setup is compatible!"
else
    echo -e "${YELLOW}⚠ System may work but not optimal${NC}"
    echo "Consider upgrading to Python 3.9 if possible"
fi

if [[ $CONFLICTS -gt 0 ]]; then
    echo ""
    echo -e "${YELLOW}⚠ $CONFLICTS potential package conflicts detected${NC}"
    echo "You may want to:"
    echo "  - Remove conflicting packages first"
    echo "  - Use virtual environment"
    echo "  - Or proceed with caution"
fi

echo ""

# Package download recommendations
echo "=== Package Download Recommendations ==="
echo "For your Python 3.9 Pi 3 setup, use these package versions:"
echo ""
echo "numpy>=1.19.5,<1.25.0  (Python 3.9 compatible)"
echo "opencv-python-headless==4.6.0.66"
echo "pygame==2.1.2"
echo "Pillow>=8.3.0,<11.0.0"
echo "smbus2==0.4.2"
echo "gpiozero==1.6.2"
echo ""
echo "These are automatically selected by download_pi3_packages.sh"

echo ""
echo "=============================================="
echo "Python 3.9 Compatibility Check Complete"
echo "=============================================="