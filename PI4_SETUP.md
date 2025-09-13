# Pi 4 Setup Guide for ECSE Robot System

## Hardware Requirements
- Raspberry Pi 4 (64-bit OS)
- Python 3.9 (matches the wheel files)
- Offline installation capability

## Quick Setup

### 1. Copy Files to Pi 4
Transfer the entire `ECSE-Comp-2025-` folder to your Raspberry Pi 4.

### 2. Install Dependencies
Run the installation script:
```bash
chmod +x install_pi4.sh
./install_pi4.sh
```

### 3. Run the Robot System
```bash
cd Software
python3 integrated_robot_system.py
```

## Key Optimizations for Pi 4

### lgpio Integration
- Upgraded from `RPi.GPIO` to `lgpio` for better performance
- Optimized for Pi 4's GPIO capabilities
- Proper resource management and cleanup

### Sensor Improvements
- Added timeout handling for ultrasonic sensor
- Distance validation (2-400cm range)
- Better error handling and recovery

### Servo Control
- Custom software PWM implementation
- Multiple pulse sending for stable positioning
- Angle tracking and validation
- Pi 4 optimized timing

### Dependency Management
- Graceful fallback for missing modules
- Mock implementations for development
- Offline wheel file installation

## Wheel Files Included
- `lgpio-0.2.2.0` - GPIO control library
- `numpy-1.24.4` - Numerical computing
- `opencv_python-4.12.0.88` - Computer vision
- `pygame-2.6.1` - Audio functionality
- `smbus2-0.5.0` - I2C communication
- `pillow-11.3.0` - Image processing

## Hardware Connections
- **Ultrasonic Sensor**: TRIG=GPIO17, ECHO=GPIO27
- **Servo 1**: GPIO18 (BOARD pin 12)
- **Servo 2**: GPIO27 (BOARD pin 13)
- **I2C LCD**: Address 0x27 on I2C bus 1

## Troubleshooting

### Permission Issues
```bash
sudo usermod -a -G gpio pi
sudo reboot
```

### GPIO Access
Ensure the user is in the `gpio` group and lgpio permissions are correct.

### Mock Mode
The system automatically falls back to mock mode if GPIO hardware isn't available, making it safe for development.

## Performance Notes
- Optimized for Pi 4's 1.5GHz ARM Cortex-A72 processor
- Efficient GPIO operations with lgpio
- Reduced CPU overhead compared to RPi.GPIO
- Better timing accuracy for servo control