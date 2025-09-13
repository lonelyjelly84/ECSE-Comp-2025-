"""
ECSE Robot Vision Navigation System
RPi Camera → OpenCV → PyTorch → Movement Commands
"""

import cv2
import torch
import numpy as np
import time
import os
import sys
from typing import List, Tuple
from dataclasses import dataclass

# Add firmware path to access robot controller
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'Firmware'))
from robot_control import ECSERobot


@dataclass
class Detection:
    """Simple detection result"""
    name: str
    confidence: float
    center: Tuple[int, int]
    area: int


class CameraCapture:
    """Handle RPi camera capture"""
    
    def __init__(self, resolution=(640, 480)):
        self.resolution = resolution
        self.cap = None
        
    def initialize(self):
        """Initialize camera"""
        try:
            # Try RPi camera first, then USB fallback
            self.cap = cv2.VideoCapture(0)
            if not self.cap.isOpened():
                return False
                
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[0])
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])
            print(f"Camera initialized: {self.resolution}")
            return True
            
        except Exception as e:
            print(f"Camera init error: {e}")
            return False
    
    def get_frame(self):
        """Capture single frame"""
        if not self.cap:
            return None
            
        ret, frame = self.cap.read()
        return frame if ret else None
    
    def release(self):
        """Release camera"""
        if self.cap:
            self.cap.release()


class ObjectDetector:
    """PyTorch object detection"""
    
    def __init__(self):
        self.model = None
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        
    def load_model(self):
        """Load YOLO model (offline version)"""
        try:
            # Try to load offline model first
            offline_model_path = "models/yolov5s_full.pt"
            if os.path.exists(offline_model_path):
                print("Loading offline YOLO model...")
                self.model = torch.load(offline_model_path, map_location=self.device)
                self.model.eval()
                print(f"Offline YOLO model loaded on {self.device}")
                return True
            else:
                # Fallback to online download (requires internet)
                print("Offline model not found, downloading...")
                self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
                self.model.to(self.device)
                self.model.eval()
                print(f"Online YOLO model loaded on {self.device}")
                return True
        except Exception as e:
            print(f"Model load error: {e}")
            return False
    
    def detect(self, frame) -> List[Detection]:
        """Detect objects in frame"""
        if not self.model:
            return []
            
        try:
            results = self.model(frame)
            detections = []
            
            for *xyxy, conf, cls in results.xyxy[0].cpu().numpy():
                if conf >= 0.5:  # Confidence threshold
                    x1, y1, x2, y2 = map(int, xyxy)
                    center = ((x1 + x2) // 2, (y1 + y2) // 2)
                    area = (x2 - x1) * (y2 - y1)
                    name = self.model.names[int(cls)]
                    
                    detections.append(Detection(
                        name=name,
                        confidence=conf,
                        center=center,
                        area=area
                    ))
            
            return detections
            
        except Exception as e:
            print(f"Detection error: {e}")
            return []


class NavigationController:
    """Simple navigation logic"""
    
    def __init__(self, robot: ECSERobot):
        self.robot = robot
        self.frame_center = 320  # Assuming 640x480
        
    def decide_movement(self, detections: List[Detection]) -> str:
        """Decide movement based on detections"""
        if not detections:
            return "forward"
        
        # Find largest detection (closest object)
        largest = max(detections, key=lambda x: x.area)
        
        # Simple logic: avoid obstacles
        if largest.area > 10000:  # Large obstacle
            obj_x = largest.center[0]
            
            if obj_x < self.frame_center - 50:
                return "turn_right"  # Object on left, turn right
            elif obj_x > self.frame_center + 50:
                return "turn_left"   # Object on right, turn left
            else:
                return "turn_left"   # Object ahead, default turn left
        
        return "forward"
    
    def execute_movement(self, action: str):
        """Execute movement command"""
        if action == "forward":
            self.robot.forward()
        elif action == "backward":
            self.robot.backward()
        elif action == "turn_left":
            self.robot.turn_left()
        elif action == "turn_right":
            self.robot.turn_right()
        elif action == "stop":
            self.robot.stop()


class VisionNavigator:
    """Main navigation system"""
    
    def __init__(self):
        self.camera = CameraCapture()
        self.detector = ObjectDetector()
        self.robot = ECSERobot()
        self.controller = NavigationController(self.robot)
        self.running = False
        
    def initialize(self):
        """Initialize all systems"""
        print("Initializing ECSE Robot Vision Navigator...")
        
        if not self.camera.initialize():
            print("Camera initialization failed")
            return False
            
        if not self.detector.load_model():
            print("Model loading failed")
            return False
            
        print("System initialized successfully")
        return True
    
    def run(self, max_iterations=50):
        """Main navigation loop"""
        self.running = True
        iteration = 0
        
        print(f"Starting navigation (max {max_iterations} iterations)")
        
        try:
            while self.running and iteration < max_iterations:
                # Capture frame
                frame = self.camera.get_frame()
                if frame is None:
                    print("Frame capture failed")
                    continue  # Retry failed operations
                
                # Detect objects
                detections = self.detector.detect(frame)
                
                # Decide movement
                action = self.controller.decide_movement(detections)
                
                # Execute movement
                self.controller.execute_movement(action)
                
                # Log
                obj_names = [d.name for d in detections] if detections else []
                print(f"Step {iteration + 1}: {action} | Objects: {obj_names}")
                
                iteration += 1
                time.sleep(0.5)  # Brief pause
                
        except KeyboardInterrupt:
            print("\nStopped by user")
        finally:
            self.stop()
    
    def stop(self):
        """Stop and cleanup"""
        self.running = False
        self.robot.stop()
        self.camera.release()
        print("System stopped")


def main():
    """Development/testing entry point - use main.py for normal operation"""
    print("Vision Navigation - Development Mode")
    print("For normal operation, run: python main.py")
    
    navigator = VisionNavigator()
    
    if navigator.initialize():
        navigator.run(max_iterations=30)
    else:
        print("Failed to initialize system")


if __name__ == "__main__":
    main()
