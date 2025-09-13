# ECSE-Comp-2025-

This repository contains the complete ECSE robot system with face recognition, arm control, and sensor integration for Raspberry Pi.

## Project Structure

```
├── Software/               # Main robot control software
│   ├── integrated_robot_system.py    # Main system integration
│   ├── face_hello.py                 # Face recognition system
│   ├── requirements.txt              # Python dependencies
│   └── known_faces/                  # Face recognition training data
├── Firmware/               # Hardware control modules
│   └── robot_control.py             # Servo and GPIO control
├── PCB/                    # Circuit board designs
├── Archive/                # Legacy/experimental code
└── resources/              # Audio and media files
```

## Dependencies

### Required Python Packages
Install with: `pip install -r Software/requirements.txt`

- **opencv-python** (>=4.8.0) - Computer vision and face recognition
- **numpy** (>=1.24.0) - Numerical computing
- **smbus2** (>=0.4.0) - I2C communication for sensors/displays  
- **pygame** (>=2.1.0) - Audio functionality
- **RPi.GPIO** (>=0.7.1) - Raspberry Pi GPIO control

### System Requirements
- Raspberry Pi (3B+ or newer recommended)
- Python 3.8+
- Camera module or USB camera
- I2C-enabled LCD display
- Servo motors for arm control

## Quick Start

1. **Install dependencies:**
   ```bash
   cd Software/
   pip install -r requirements.txt
   ```

2. **Add known faces:**
   - Place face photos (jpg/png) in `Software/known_faces/`
   - Name files as `person_name.jpg`

3. **Run the system:**
   ```bash
   python3 integrated_robot_system.py
   ```

## Hardware Setup

### Connections
- **Camera**: Connect to Pi camera port or USB
- **Servos**: GPIO pins for arm control (see robot_control.py)
- **I2C Display**: SDA/SCL pins for status display
- **Sensors**: Additional GPIO pins for navigation

### I2C Configuration
Enable I2C interface:
```bash
sudo raspi-config
# Interface Options → I2C → Enable
```

## Features

- **Face Recognition**: Real-time face detection and recognition
- **Arm Control**: Servo-based robotic arm movements
- **Audio Feedback**: Robot personality with sound effects
- **Sensor Integration**: Ultrasonic and other sensors
- **State Machine**: Coordinated behavior control
- **I2C Display**: Status and information display

## Troubleshooting

### Common Issues
- **Camera not found**: Check camera connection and enable camera interface
- **I2C errors**: Verify I2C is enabled and devices are connected
- **Import errors**: Ensure all dependencies are installed
- **GPIO permissions**: Run with `sudo` if needed for GPIO access

### Offline Installation
For systems without internet, download packages with:
```bash
pip download -r requirements.txt --platform linux_armv7l --only-binary=:all:
```
Then transfer and install the .whl files on the Pi. 
