#!/bin/bash
# Validation script for offline Pi 3 installation
# Tests all components without requiring internet
# Validates Python 3.9 compatibility

echo "=============================================="
echo "Pi 3 Offline Installation Validator (Python 3.9)"
echo "=============================================="

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Test counters
TESTS_PASSED=0
TESTS_FAILED=0

# Function to run test
run_test() {
    local test_name="$1"
    local test_command="$2"
    
    echo -n "Testing $test_name... "
    
    if eval "$test_command" &>/dev/null; then
        echo -e "${GREEN}✓ PASS${NC}"
        ((TESTS_PASSED++))
        return 0
    else
        echo -e "${RED}✗ FAIL${NC}"
        ((TESTS_FAILED++))
        return 1
    fi
}

# Function to run test with output
run_test_with_output() {
    local test_name="$1"
    local test_command="$2"
    
    echo "Testing $test_name..."
    
    if result=$(eval "$test_command" 2>&1); then
        echo -e "${GREEN}✓ PASS${NC}: $result"
        ((TESTS_PASSED++))
        return 0
    else
        echo -e "${RED}✗ FAIL${NC}: $result"
        ((TESTS_FAILED++))
        return 1
    fi
}

echo "Running validation tests..."
echo ""

# Test 1: Check Pi 3 hardware
echo "=== Hardware Tests ==="
run_test_with_output "Pi Model Detection" "cat /proc/device-tree/model"
run_test_with_output "Architecture Check" "uname -m"
run_test "CPU Temperature" "vcgencmd measure_temp"

echo ""

# Test 2: Check offline packages
echo "=== Package Availability Tests ==="
run_test "Offline packages directory" "[ -d 'rpi3_packages' ]"
run_test "NumPy wheel file" "ls rpi3_packages/numpy*.whl"
run_test "OpenCV wheel file" "ls rpi3_packages/opencv*.whl"
run_test "Pygame wheel file" "ls rpi3_packages/pygame*.whl"
run_test "SetupTools wheel file" "ls rpi3_packages/setuptools*.whl"

echo ""

# Test 3: Python version compatibility
echo "=== Python 3.9 Compatibility Tests ==="
run_test_with_output "Python Version Check" "python3 -c 'import sys; print(f\"{sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}\")'"
run_test "Python 3.9 Specific" "python3 -c 'import sys; assert sys.version_info.major == 3 and sys.version_info.minor == 9'"

# Test 4: Python import tests
echo "=== Python 3.9 Import Tests ==="
run_test_with_output "Python 3.9 Version" "python3 --version"
run_test "NumPy import" "python3 -c 'import numpy; print(f\"NumPy {numpy.__version__} (Python {numpy.version.version})\")'"
run_test "OpenCV import" "python3 -c 'import cv2; print(f\"OpenCV {cv2.__version__}\")'"
run_test "Pygame import" "python3 -c 'import pygame; print(f\"Pygame {pygame.version.ver}\")'"
run_test "SMBus2 import" "python3 -c 'import smbus2; print(\"SMBus2 OK\")'"
run_test "GPIOZero import" "python3 -c 'import gpiozero; print(\"GPIOZero OK\")'"

echo ""

# Test 4: Hardware interface tests
echo "=== Hardware Interface Tests ==="
run_test "I2C interface" "i2cdetect -y 1"
run_test "Camera interface" "vcgencmd get_camera | grep 'detected=1'"
run_test "GPIO access" "[ -d '/sys/class/gpio' ]"

echo ""

# Test 5: Robot-specific tests
echo "=== Robot System Tests ==="
run_test "Robot main file" "[ -f 'robot_name.py' ]"
run_test "Face detection module" "[ -f 'Software/face_hello.py' ]"
run_test "Audio resources" "[ -d 'resources/audio' ]"

# Test Pi 3 optimizations
echo ""
echo "=== Pi 3 Optimization Tests ==="
python3 -c "
import sys
sys.path.append('.')

# Test Pi 3 detection
try:
    with open('/proc/device-tree/model', 'r') as f:
        model = f.read()
        pi3_detected = 'Raspberry Pi 3' in model
        print(f'Pi 3 detection: {\"ENABLED\" if pi3_detected else \"DISABLED\"}')
        
    # Test optimized imports
    try:
        exec(open('robot_name.py').read())
        print('Robot script syntax: VALID')
    except Exception as e:
        print(f'Robot script syntax: ERROR - {e}')
        
except Exception as e:
    print(f'Pi model detection: ERROR - {e}')
"

echo ""
echo "=============================================="
echo "Validation Results"
echo "=============================================="
echo -e "Tests passed: ${GREEN}$TESTS_PASSED${NC}"
echo -e "Tests failed: ${RED}$TESTS_FAILED${NC}"

if [ $TESTS_FAILED -eq 0 ]; then
    echo -e "\n${GREEN}🎉 ALL TESTS PASSED!${NC}"
    echo "Your offline Pi 3 setup is ready!"
    echo ""
    echo "To start the robot:"
    echo "  python3 robot_name.py"
    exit 0
else
    echo -e "\n${YELLOW}⚠ Some tests failed.${NC}"
    echo "Check the failed tests above and:"
    echo "1. Ensure all packages were transferred correctly"
    echo "2. Run the installation script again"
    echo "3. Check hardware connections"
    echo ""
    echo "For help, see OFFLINE_SETUP.md"
    exit 1
fi