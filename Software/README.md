# ECSE Robot Vision Navigation

Simple vision-based navigation system for the ECSE Robot.

## Core Flow
```
RPi Camera → OpenCV → PyTorch YOLO → Movement Decision → Robot Commands
```

## Files
- `vision_navigation.py` - Main navigation system
- `models/` - Pre-downloaded YOLO model (offline)
- `requirements.txt` - Dependencies
- `../Firmware/robot_control.py` - Robot movement functions (hardware control)

## Installation
```bash
pip install opencv-python torch torchvision numpy
```

## Usage
```bash
python vision_navigation.py
```

## Offline Model
The YOLO model is pre-downloaded in the `models/` folder for offline use:
- `models/yolov5s_full.pt` - Complete YOLO model
- `models/model_info.txt` - Model details (80 object classes)

## Movement Logic
- **No objects detected**: Move forward
- **Object on left**: Turn right
- **Object on right**: Turn left  
- **Object ahead**: Turn left (default)

## Robot Functions
The system uses the robot controller from `../Firmware/robot_control.py`:
- `forward()` - Move forward
- `backward()` - Move backward  
- `turn_left()` - Turn left
- `turn_right()` - Turn right
- `stop()` - Stop movement

## Deployment
1. Copy entire project folder to Raspberry Pi (including both Software/ and Firmware/)
2. Install dependencies: `pip install -r Software/requirements.txt`
3. Update robot functions in `Firmware/robot_control.py` for your actual hardware
4. Run from Software/: `python vision_navigation.py`