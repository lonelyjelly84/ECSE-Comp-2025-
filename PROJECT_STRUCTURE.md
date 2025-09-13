# ECSE Robot Project - Clean Structure

## 📁 Current Project Structure

### 🚀 **Active Files (Main System)**
```
Software/
├── integrated_robot_system.py    # 🤖 MAIN ROBOT SYSTEM - Run this!
├── face_hello.py                 # 👁️ Face recognition core
└── requirements.txt              # 📦 Dependencies

Firmware/
└── robot_control.py              # 🦾 Servo/arm control core

Archive/
├── sg90_robot.py                 # 📜 Original groupmate file
├── arm_face_controller.py        # 📜 Your original arm controller  
├── I2C_LCD_DISPLAY.py           # 📜 Separate LCD module
├── vision_navigation.py          # 📜 Old navigation system
├── sensor_navigation.py          # 📜 Old navigation system
└── main.py                       # 📜 Old navigation orchestrator
```

## 🎯 **How to Use**

### **Main Robot System:**
```bash
cd Software
python3 integrated_robot_system.py
```

### **Test Individual Components:**
```bash
# Test face recognition only
python3 face_hello.py

# Test arm movements only  
cd ../Firmware
python3 robot_control.py
```

## 🤖 **System Features**

**integrated_robot_system.py** combines:
- ✅ Face recognition AI (from face_hello.py)
- ✅ Arm movements (from robot_control.py)  
- ✅ LCD display & personality (from sg90_robot.py)
- ✅ Ultrasonic sensors (from sg90_robot.py)
- ✅ Audio/music system (from sg90_robot.py)
- ✅ State machine (IDLE → FRIEND_DETECTED → TOO_CLOSE)

## 📦 **Dependencies**

Install on Raspberry Pi:
```bash
sudo apt update
sudo apt install python3-opencv python3-pip
pip3 install opencv-python RPi.GPIO pygame smbus2 numpy
```

## 🎭 **Robot Behavior**

1. **IDLE**: Lonely, plays sad music, droopy arm
2. **FRIEND_DETECTED**: Happy, plays energetic music, waves enthusiastically  
3. **TOO_CLOSE**: Warning state, defensive gestures

## 🗂️ **File Purposes**

| File | Purpose | Status |
|------|---------|--------|
| `integrated_robot_system.py` | Complete robot system | ✅ Active |
| `face_hello.py` | Face recognition engine | ✅ Active |
| `robot_control.py` | Servo control engine | ✅ Active |
| Archive files | Historical/reference | 📦 Archived |

---

**🎯 Main Entry Point: `Software/integrated_robot_system.py`**
