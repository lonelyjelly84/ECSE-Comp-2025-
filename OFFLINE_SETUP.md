# 🔒 Offline Pi 3 Setup Guide (Python 3.9)

This guide provides complete instructions for setting up the ECSE Robot System on a **Raspberry Pi 3 64-bit** with **Python 3.9** that has **NO INTERNET ACCESS**.

## 📋 Overview

Since your Pi 3 cannot connect to the internet, we use a **two-machine process**:

1. **Internet Machine**: Downloads Python 3.9 compatible packages 
2. **Offline Pi 3**: Installs from local files

## ⚙️ Python 3.9 Requirements

### **Verify Your Pi 3 Setup**
```bash
# Check Python version (should be 3.9.x)
python3 --version

# Check architecture (should be aarch64)  
uname -m

# Check Pi model
cat /proc/device-tree/model
```

### **Expected Output:**
```
Python 3.9.2 (default, Feb 28 2021, 17:03:44)
aarch64
Raspberry Pi 3 Model B Plus Rev 1.3
```

## 🖥️ Part 1: Download Packages (Internet-Connected Machine)

### Requirements
- Any computer with **Python 3** and **internet access**
- **USB drive** or **SD card** for file transfer
- About **200MB** free space

### Steps

1. **Clone/Download the project** to the internet-connected machine
2. **Run the download script**:
   ```bash
   chmod +x download_pi3_packages.sh
   ./download_pi3_packages.sh
   ```

3. **Verify downloads**:
   ```bash
   ls -la rpi3_packages/
   # Should show .whl files for all packages
   ```

4. **Copy to transfer media**:
   ```bash
   # Copy the entire project including rpi3_packages to USB/SD
   cp -r ECSE-Comp-2025- /media/usb_drive/
   ```

## 📦 Package Details (Python 3.9 Optimized)

The download script gets these Python 3.9 compatible packages:

| Package | Version | Python 3.9 | Size | Purpose |
|---------|---------|-------------|------|---------|
| `numpy` | 1.19.5-1.24.x | ✅ Compatible | ~15MB | Numerical computing |
| `opencv-python-headless` | 4.6.0.66 | ✅ Compatible | ~90MB | Computer vision (no GUI) |
| `pygame` | 2.1.2 | ✅ Compatible | ~20MB | Audio playback |
| `Pillow` | 8.3.0-10.x | ✅ Compatible | ~10MB | Image processing |
| `smbus2` | 0.4.2 | ✅ Compatible | ~0.1MB | I2C communication |
| `gpiozero` | 1.6.2 | ✅ Compatible | ~1MB | GPIO control |
| `setuptools` | 68.2.2 | ✅ Compatible | ~2MB | Package tools |

**Total**: ~140MB

### **Python 3.9 Specific Notes:**
- NumPy versions >1.25 drop Python 3.9 support
- OpenCV 4.6.x is last stable version for Python 3.9  
- Pillow <11.0 maintains Python 3.9 compatibility

## 🔌 Part 2: Transfer to Pi 3

### Method 1: USB Drive Transfer
```bash
# On internet machine - copy to USB
cp -r ECSE-Comp-2025- /media/usb_drive/

# On Pi 3 - copy from USB
cp -r /media/usb_drive/ECSE-Comp-2025- ~/
cd ~/ECSE-Comp-2025-
```

### Method 2: SD Card Pre-Installation
```bash
# Before first boot, mount Pi 3's SD card on internet machine
# Copy to /home/pi/ directory on SD card
cp -r ECSE-Comp-2025- /media/sd_card/home/pi/
```

### Method 3: Direct SD Writing
```bash
# Flash Pi OS to SD card
# Mount SD card on internet machine
# Copy project to /home/pi/ before first boot
```

## 🤖 Part 3: Install on Offline Pi 3

### Prerequisites Check
```bash
# Run Python 3.9 compatibility check
chmod +x check_python39.sh
./check_python39.sh

# Verify Pi 3 model
cat /proc/device-tree/model

# Check Python 3.9 specifically
python3 -c "import sys; assert sys.version_info.major==3 and sys.version_info.minor==9; print('Python 3.9 confirmed')"

# Check architecture (should be aarch64)
uname -m

# Verify offline packages exist  
ls -la ECSE-Comp-2025-/rpi3_packages/
```

### Installation
```bash
cd ECSE-Comp-2025-

# Make scripts executable
chmod +x *.sh

# Run installation (auto-detects offline mode)
./install_pi3.sh
```

The script will automatically:
1. **Detect no internet** connection
2. **Switch to offline mode**
3. **Use local packages** only
4. **Configure hardware** interfaces
5. **Test installation**

## 🔧 Manual Installation (Alternative)

If the automatic script fails:

```bash
cd ECSE-Comp-2025-/rpi3_packages

# Install packages manually
pip3 install --no-index --find-links . setuptools*.whl
pip3 install --no-index --find-links . wheel*.whl
pip3 install --no-index --find-links . numpy*.whl
pip3 install --no-index --find-links . opencv_python_headless*.whl
pip3 install --no-index --find-links . pygame*.whl
pip3 install --no-index --find-links . smbus2*.whl
pip3 install --no-index --find-links . gpiozero*.whl
```

## 🏗️ Hardware Setup (No Internet Required)

### Enable Interfaces
```bash
sudo raspi-config nonint do_i2c 0      # Enable I2C
sudo raspi-config nonint do_camera 0   # Enable Camera
sudo reboot                             # Reboot to apply
```

### Physical Connections
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
│ Pin 4  (5V)      → VCC (if needed)  │
└─────────────────────────────────────┘
```

## ✅ Testing Installation

### Quick Test
```bash
cd ECSE-Comp-2025-
python3 -c "
import numpy as np
import cv2  
import pygame
import smbus2
import gpiozero
print('All packages imported successfully!')
"
```

### Hardware Test
```bash
# Test I2C
i2cdetect -y 1

# Test camera
python3 -c "import cv2; print('Camera:', cv2.VideoCapture(0).isOpened())"

# Test GPIO
python3 -c "from gpiozero import LED; print('GPIO ready')"
```

### Full Robot Test
```bash
# Start the robot system
python3 robot_name.py

# Should see output like:
# "Detected hardware: Pi3"
# "Pi 3 optimization: ENABLED"
# "Camera settings: 320x240 @ 10fps"
```

## 📊 Expected Performance

| Feature | Pi 3 Offline | Notes |
|---------|--------------|-------|
| **Boot Time** | ~60s | Slower due to offline setup |
| **Face Detection** | 1-2s | Optimized for Pi 3 CPU |
| **State Changes** | 2-3s | Stable operation |
| **Memory Usage** | ~400MB | Within Pi 3 limits |

## 🔧 Troubleshooting

### Package Installation Issues
```bash
# If pip install fails, try upgrading pip first
python3 -m pip install --upgrade pip --no-index --find-links rpi3_packages/

# Check available packages
ls -la rpi3_packages/*.whl

# Manually install each package
pip3 install --no-index --find-links rpi3_packages/ package_name.whl
```

### Hardware Issues
```bash
# I2C not working
sudo raspi-config  # Interface Options → I2C → Enable

# Camera not detected  
sudo raspi-config  # Interface Options → Camera → Enable
sudo reboot

# GPIO permission issues
sudo usermod -a -G gpio pi
sudo reboot
```

### Performance Issues
```bash
# Check CPU temperature
vcgencmd measure_temp

# Monitor system resources
top -p $(pgrep python3)

# Check for throttling
vcgencmd get_throttled
```

## 📂 File Structure After Setup

```
ECSE-Comp-2025-/
├── robot_name.py              # Main robot script
├── install_pi3_offline.sh     # Offline installer
├── rpi3_packages/             # Local Python packages
│   ├── numpy-*.whl
│   ├── opencv_python_headless-*.whl
│   └── ...
├── Software/
│   ├── face_hello.py
│   └── integrated_robot_system.py
└── resources/
    └── audio/
```

## 🔄 Updates and Maintenance

Since the Pi 3 is offline:

1. **Code updates**: Transfer via USB/SD card
2. **Package updates**: Re-run download script on internet machine
3. **System updates**: Not possible offline (Pi OS will work as-is)

## 🎯 Success Indicators

You'll know the setup worked when:

- ✅ All Python packages import without errors
- ✅ Camera detection returns `True`
- ✅ I2C shows device on bus 1
- ✅ Robot script starts with "Pi3 optimization: ENABLED"
- ✅ Face detection works (slower but functional)

## 📞 Support

If you encounter issues:

1. **Check logs**: Look for error messages during installation
2. **Verify files**: Ensure all .whl files transferred correctly
3. **Test components**: Test each hardware component individually
4. **Review connections**: Double-check GPIO wiring

---

**Remember**: The Pi 3 will work completely offline once set up. All robot functionality is self-contained with no internet dependencies!