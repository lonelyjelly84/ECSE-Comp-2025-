# ECSE Robot Navigation System

**Intelligent robot navigation with automatic mode selection**

## How It Works

```
Robot Start → Check Camera → AI Vision OR Sensor Fallback → Navigate
```

### **Primary Mode: Vision Navigation**
- **Camera**: RPi Camera Module via OpenCV
- **AI Processing**: PyTorch YOLOv5 object detection (offline)
- **Smart Movement**: Intelligent obstacle avoidance

### **Fallback Mode: Sensor Navigation**  
- **Sensors**: HC-SR04 ultrasonic distance sensors
- **Movement**: Distance-based obstacle avoidance
- **Reliability**: Works without camera dependencies

## Quick Start

### **Main Program:**
```bash
python main.py
```
The system automatically detects your hardware and chooses the best navigation mode!

### **Individual Modules (Optional):**
```bash
python vision_navigation.py # Camera mode only
python sensor_navigation.py # Sensor mode only
```

## Files
- **`main.py`** - **MAIN PROGRAM** (run this!)
- `vision_navigation.py` - Camera + AI navigation module
- `sensor_navigation.py` - Ultrasonic sensor navigation module
- `models/yolov5s_full.pt` - Pre-downloaded YOLO model (14MB, offline)
- `requirements.txt` - Python dependencies
- `../Firmware/robot_control.py` - Robot hardware control

## Installation on Raspberry Pi
```bash
# 1. Copy project to RPi
# 2. Install dependencies
pip install -r requirements.txt

# 3. Run the system
python main.py
```

## System Intelligence
- **Auto-detects camera availability**
- **Checks all dependencies automatically**  
- **Falls back gracefully if hardware unavailable**
- **Works offline with pre-downloaded AI model**

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