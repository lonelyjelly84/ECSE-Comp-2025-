# Raspberry Pi 3 64-bit Setup Guide

This guide provides instructions for adapting the ECSE Robot System to run on **Raspberry Pi 3 64-bit** hardware.

## 🔧 Hardware Differences: Pi 3 vs Pi 4

| Component | Pi 3 64-bit | Pi 4 64-bit | Impact |
|-----------|-------------|-------------|---------|
| **CPU** | ARM Cortex-A53 @ 1.2GHz | ARM Cortex-A72 @ 1.5GHz | ~2-3x slower processing |
| **RAM** | 1GB LPDDR2 | 2-8GB LPDDR4 | Limited memory for OpenCV |
| **GPIO** | 40-pin header | 40-pin header | ✅ Compatible |
| **Camera** | CSI connector | CSI connector | ✅ Compatible |
| **I2C** | Hardware I2C | Hardware I2C | ✅ Compatible |

## 📋 Prerequisites

### 1. Operating System
- **Raspberry Pi OS 64-bit** (Bullseye or newer)
- **Not** the 32-bit version (for better NumPy/OpenCV performance)

### 2. Check Your Pi Model
```bash
# Verify you have Pi 3
cat /proc/device-tree/model

# Check architecture (should show aarch64)
uname -m

# Check OS version
cat /etc/os-release
```

## 🚀 Installation

### Method 1: Automatic Installation (Recommended)
```bash
# Make script executable
chmod +x install_pi3.sh

# Run Pi 3 optimized installation
./install_pi3.sh
```

### Method 2: Manual Installation
```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install system dependencies
sudo apt install -y python3-pip python3-dev i2c-tools python3-smbus
sudo apt install -y libatlas-base-dev libhdf5-dev

# Enable I2C and Camera
sudo raspi-config nonint do_i2c 0
sudo raspi-config nonint do_camera 0

# Install Python packages (optimized versions)
pip3 install numpy==1.21.6
pip3 install opencv-python-headless==4.6.0.66
pip3 install smbus2==0.4.2
pip3 install pygame==2.1.2
pip3 install gpiozero
```

## ⚙️ Pi 3 Optimizations Applied

### 1. **Camera Settings**
- **Resolution**: 320x240 (vs 640x480 on Pi 4)
- **FPS**: 10 fps (vs 15 fps on Pi 4)
- **Processing**: Smaller detection frames (120x90)

### 2. **Face Detection**
- **Scale Factor**: 1.3 (less sensitive, faster)
- **Min Neighbors**: 2 (reduced accuracy for speed)
- **Check Intervals**: 1.5s (vs 1.0s on Pi 4)

### 3. **Performance Tuning**
- Longer camera warm-up time (150ms vs 100ms)
- Additional processing delays
- Reduced servo movement frequency
- Optimized OpenCV parameters

## 🔌 Hardware Connections

Connections are **identical** to Pi 4 setup:

```
┌─────────────────────────────────────┐
│  Raspberry Pi 3 (40-pin header)    │
├─────────────────────────────────────┤
│ Pin 11 (GPIO 17) → Ultrasonic TRIG │
│ Pin 13 (GPIO 27) → Ultrasonic ECHO │
│ Pin 33 (GPIO 13) → Servo Signal    │
│ Pin 3  (SDA)     → LCD SDA          │
│ Pin 5  (SCL)     → LCD SCL          │
│ Pin 6  (GND)     → Common Ground    │
│ Pin 4  (5V)      → Power (if needed)│
└─────────────────────────────────────┘
```

## 🎯 Running the Robot

### Start the System
```bash
cd /path/to/ECSE-Comp-2025-
python3 robot_name.py
```

### Expected Performance
- **Face Detection**: ~1-2 seconds per check
- **State Transitions**: Slower but stable
- **Servo Response**: Slightly delayed but smooth

## 📊 Monitoring Pi 3 Performance

### Check System Resources
```bash
# CPU temperature (should stay below 70°C)
vcgencmd measure_temp

# CPU usage
top -p $(pgrep python3)

# Memory usage
free -h

# Check for throttling
vcgencmd get_throttled
```

### Performance Tips
1. **Enable GPU memory split**: `sudo raspi-config` → Advanced → Memory Split → 128
2. **Overclock safely**: `sudo raspi-config` → Advanced → Overclock → Modest (1000MHz)
3. **Use heatsink**: Pi 3 generates more heat under load
4. **Good power supply**: Use quality 2.5A+ power adapter

## 🔧 Troubleshooting

### Common Issues

#### 1. **Slow Face Detection**
```bash
# Check if Pi 3 optimizations are active
grep "Pi3 mode: True" robot_output.log
```

#### 2. **Camera Errors**
```bash
# Test camera independently
raspistill -o test.jpg -w 320 -h 240

# Check camera in Python
python3 -c "import cv2; print(cv2.VideoCapture(0).isOpened())"
```

#### 3. **Memory Issues**
```bash
# Monitor memory during operation
watch -n 1 'free -h'

# Increase swap if needed
sudo dphys-swapfile swapoff
sudo nano /etc/dphys-swapfile  # CONF_SWAPSIZE=1024
sudo dphys-swapfile setup
sudo dphys-swapfile swapon
```

#### 4. **Thermal Throttling**
```bash
# Check throttling status
vcgencmd get_throttled

# 0x0 = No throttling
# 0x50000 = Currently throttled
# Install cooling or reduce load
```

## 🎛️ Configuration Files

### Pi 3 Specific Settings
The robot automatically detects Pi 3 and applies optimizations:

```python
PI_MODEL = detect_pi_model()  # Returns 'Pi3', 'Pi4', or 'Unknown'

# Optimizations applied when PI_MODEL == 'Pi3':
# - Reduced camera resolution
# - Longer processing intervals  
# - Simplified face detection
# - Additional delays
```

## 📈 Performance Comparison

| Metric | Pi 3 64-bit | Pi 4 64-bit |
|--------|-------------|-------------|
| **Boot Time** | ~45s | ~30s |
| **Face Detection** | 1-2s | 0.5s |
| **State Change** | 2-3s | 1s |
| **Memory Usage** | ~400MB | ~300MB |
| **CPU Load** | 60-80% | 20-40% |

## ✅ Validation

### Test Face Detection
```bash
python3 -c "
import sys
sys.path.append('Software')
from pi3_optimized import detect_face_quick, PI3_DETECTED
print(f'Pi 3 mode: {PI3_DETECTED}')
for i in range(3):
    result = detect_face_quick()
    print(f'Test {i+1}: {\"Face\" if result else \"No face\"}')
"
```

### Full System Test
```bash
# Run for 60 seconds and monitor
timeout 60 python3 robot_name.py
```

## 📚 Additional Resources

- [Raspberry Pi 3 Official Documentation](https://www.raspberrypi.org/documentation/hardware/raspberrypi/README.md)
- [OpenCV Pi 3 Optimization Guide](https://www.pyimagesearch.com/2017/10/09/optimizing-opencv-on-the-raspberry-pi/)
- [Pi 3 vs Pi 4 Performance Comparison](https://magpi.raspberrypi.org/articles/raspberry-pi-4-specs-benchmarks)

---

**Note**: Pi 3 performance will be noticeably slower than Pi 4, but the robot should function reliably with the applied optimizations.