#!/bin/bash
# Analysis of existing rpi_packages for Pi 3 compatibility
# This script checks if the current wheel files work on Pi 3 with Python 3.9

echo "=============================================="
echo "Pi 3 Compatibility Analysis of Existing Wheels"
echo "=============================================="

cd rpi_packages 2>/dev/null || {
    echo "❌ rpi_packages directory not found"
    echo "Run this from the main project directory"
    exit 1
}

echo "Analyzing existing wheel files for Pi 3 compatibility..."
echo ""

# Function to check wheel compatibility
check_wheel() {
    local file="$1"
    local component="$2"
    local issues=""
    local compatible="✅"
    
    echo "=== $component ==="
    echo "File: $file"
    
    # Check if file exists
    if [[ ! -f "$file" ]]; then
        echo "❌ File not found"
        return 1
    fi
    
    # Parse filename components
    if [[ "$file" =~ (.+)-(.+)-(.+)-(.+)-(.+)\.whl ]]; then
        package="${BASH_REMATCH[1]}"
        version="${BASH_REMATCH[2]}"
        python_tag="${BASH_REMATCH[3]}"
        abi_tag="${BASH_REMATCH[4]}"
        platform_tag="${BASH_REMATCH[5]}"
    elif [[ "$file" =~ (.+)-(.+)-(.+)-(.+)\.whl ]]; then
        package="${BASH_REMATCH[1]}"
        version="${BASH_REMATCH[2]}"
        python_tag="${BASH_REMATCH[3]}"
        platform_tag="${BASH_REMATCH[4]}"
        abi_tag="none"
    else
        echo "❌ Cannot parse wheel filename"
        return 1
    fi
    
    echo "Package: $package"
    echo "Version: $version"
    echo "Python: $python_tag"
    echo "ABI: $abi_tag"
    echo "Platform: $platform_tag"
    
    # Check Python compatibility
    case "$python_tag" in
        "cp39"|"cp37-abi3"|"py3"|"py2.py3")
            echo "✅ Python tag compatible with Python 3.9"
            ;;
        "cp310"|"cp311"|"cp312")
            echo "❌ Python tag requires Python 3.10+"
            compatible="❌"
            issues="$issues Python version mismatch;"
            ;;
        *)
            echo "⚠ Unknown Python tag: $python_tag"
            compatible="⚠"
            ;;
    esac
    
    # Check platform compatibility
    case "$platform_tag" in
        *"aarch64"*)
            echo "✅ Platform compatible with Pi 3 64-bit (aarch64)"
            ;;
        *"armv7l"*)
            echo "❌ Platform is for 32-bit ARM (Pi 3 is 64-bit)"
            compatible="❌"
            issues="$issues Architecture mismatch;"
            ;;
        *"any"*)
            echo "✅ Platform universal (any architecture)"
            ;;
        *)
            echo "⚠ Unknown platform: $platform_tag"
            compatible="⚠"
            ;;
    esac
    
    # Check specific package versions for Pi 3 suitability
    case "$package" in
        "numpy")
            if [[ "$version" > "1.25.0" ]]; then
                echo "⚠ NumPy >1.25 may drop Python 3.9 support"
                compatible="⚠"
            fi
            ;;
        "pillow"|"Pillow")
            if [[ "$version" > "11.0.0" ]]; then
                echo "⚠ Pillow >11.0 may drop Python 3.9 support"
                compatible="⚠"
            fi
            ;;
        "opencv_python")
            echo "⚠ Full OpenCV may be heavy for Pi 3 (headless version preferred)"
            compatible="⚠"
            ;;
    esac
    
    echo "Overall compatibility: $compatible"
    if [[ -n "$issues" ]]; then
        echo "Issues: $issues"
    fi
    echo ""
    
    return 0
}

# Analyze each wheel file
echo "Checking existing wheel files..."
echo ""

check_wheel "numpy-1.24.4-cp39-cp39-manylinux_2_17_aarch64.manylinux2014_aarch64.whl" "NumPy"
check_wheel "opencv_python-4.8.1.78-cp37-abi3-manylinux_2_17_aarch64.manylinux2014_aarch64.whl" "OpenCV"
check_wheel "pillow-11.3.0-cp39-cp39-manylinux2014_aarch64.manylinux_2_17_aarch64.whl" "Pillow"
check_wheel "pygame-2.6.1-cp39-cp39-manylinux_2_17_aarch64.manylinux2014_aarch64.whl" "Pygame"
check_wheel "setuptools-80.9.0-py3-none-any.whl" "SetupTools"
check_wheel "smbus2-0.5.0-py2.py3-none-any.whl" "SMBus2"
check_wheel "wheel-0.45.1-py3-none-any.whl" "Wheel"

echo "=============================================="
echo "SUMMARY"
echo "=============================================="

echo ""
echo "✅ COMPATIBLE (Ready for Pi 3):"
echo "  - numpy-1.24.4 (Python 3.9, aarch64)"
echo "  - smbus2-0.5.0 (Universal Python)"
echo "  - setuptools-80.9.0 (Universal Python)"
echo "  - wheel-0.45.1 (Universal Python)"

echo ""
echo "⚠ MOSTLY COMPATIBLE (Will work but not optimal):"
echo "  - opencv_python-4.8.1.78 (Heavy for Pi 3, prefer headless)"
echo "  - pygame-2.6.1 (Newer version, may be resource intensive)"

echo ""
echo "⚠ POTENTIAL ISSUES:"
echo "  - pillow-11.3.0 (Version 11.3 may have Python 3.9 issues)"

echo ""
echo "=============================================="
echo "RECOMMENDATIONS"
echo "=============================================="

echo ""
echo "🎯 Can you use existing wheels on Pi 3?"
echo ""
echo "✅ YES, most will work, but with caveats:"
echo ""
echo "1. ARCHITECTURE: All aarch64 wheels are compatible with Pi 3 64-bit"
echo "2. PYTHON VERSION: cp39 and abi3 wheels work with Python 3.9"
echo "3. PERFORMANCE: Some packages may be too resource-intensive"
echo ""
echo "🔧 Recommended changes for better Pi 3 performance:"
echo ""
echo "REPLACE:"
echo "  opencv_python-4.8.1.78         → opencv-python-headless-4.6.0.66"
echo "  pillow-11.3.0                  → Pillow-9.5.0 or 10.x"
echo "  pygame-2.6.1                   → pygame-2.1.2"
echo ""
echo "KEEP AS-IS:"
echo "  numpy-1.24.4                   → ✅ Good for Pi 3"
echo "  smbus2-0.5.0                   → ✅ Perfect"
echo "  setuptools/wheel               → ✅ Universal"
echo ""
echo "🚀 Quick test command:"
echo "Try installing existing wheels first, then replace problematic ones:"
echo ""
echo "cd rpi_packages"
echo "pip3 install --no-index --find-links . *.whl"
echo ""
echo "If issues occur, use the Pi 3 optimized download script instead."