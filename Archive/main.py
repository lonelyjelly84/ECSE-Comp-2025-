#!/usr/bin/env python3
"""
ECSE Robot Navigation System - Unified Main Function
Auto-prioritize camera, fallback to sensors if camera unavailable
"""

import sys
import os

def check_camera_availability():
    """Check if camera and vision dependencies are available"""
    try:
        import cv2
        # Try to initialize camera
        cap = cv2.VideoCapture(0)
        if cap.isOpened():
            ret, _ = cap.read()
            cap.release()
            if ret:
                return True, "Camera detected and working"
        cap.release()
        return False, "Camera not responding"
    except ImportError:
        return False, "OpenCV not installed"
    except Exception as e:
        return False, f"Camera error: {e}"

def check_vision_dependencies():
    """Check if all vision navigation dependencies are available"""
    try:
        import cv2
        import torch
        import torchvision
        return True, "All vision dependencies available"
    except ImportError as e:
        return False, f"Missing dependency: {e}"

def run_vision_navigation():
    """Run vision navigation system"""
    try:
        from vision_navigation import VisionNavigator
        
        print("Initializing Vision Navigation System...")
        navigator = VisionNavigator()
        
        if navigator.initialize():
            print("Vision system initialized successfully")
            navigator.run(max_iterations=50)
        else:
            print("Vision system initialization failed")
            return False
        return True
    except Exception as e:
        print(f"Vision navigation error: {e}")
        return False

def run_sensor_navigation():
    """Run sensor navigation system"""
    try:
        from sensor_navigation import SensorNavigator
        
        print("Initializing Sensor Navigation System...")
        navigator = SensorNavigator()
        
        if navigator.initialize():
            print("Sensor system initialized successfully")  
            navigator.run(max_iterations=100)
        else:
            print("Sensor system initialization failed")
            return False
        return True
    except Exception as e:
        print(f"Sensor navigation error: {e}")
        return False

def main():
    """
    Unified main function for ECSE Robot Navigation
    Auto-detects best available navigation method
    """
    print("=" * 70)
    print("ECSE ROBOT NAVIGATION SYSTEM")
    print("Intelligent Vision + Sensor Navigation")
    print("=" * 70)
    
    print("\nSystem Detection Phase...")
    
    # Check camera availability
    camera_ok, camera_msg = check_camera_availability()
    print(f"Camera Status: {camera_msg}")
    
    if camera_ok:
        # Check vision dependencies
        deps_ok, deps_msg = check_vision_dependencies()
        print(f"Vision Dependencies: {deps_msg}")
        
        if deps_ok:
            print("\nVISION NAVIGATION MODE SELECTED")
            print("Using: RPi Camera + AI Object Detection")
            print("Model: YOLOv5 (Offline)")
            
            if run_vision_navigation():
                print("\nVision navigation completed successfully")
            else:
                print("\nVision navigation failed - switching to sensor fallback...")
                if run_sensor_navigation():
                    print("\nSensor navigation completed successfully")
                else:
                    print("\nBoth navigation systems failed")
        else:
            print("\nSENSOR NAVIGATION MODE SELECTED")
            print("Reason: Vision dependencies missing")
            print("To enable vision: pip install opencv-python torch torchvision")
            
            if run_sensor_navigation():
                print("\nSensor navigation completed successfully")
            else:
                print("\nSensor navigation failed")
    else:
        print("\nSENSOR NAVIGATION MODE SELECTED")
        print("Reason: Camera unavailable")
        print("Using: HC-SR04 Ultrasonic Sensors")
        
        if run_sensor_navigation():
            print("\nSensor navigation completed successfully")
        else:
            print("\nSensor navigation failed")
    
    print("\n" + "=" * 70)
    print("ECSE Robot Navigation System Stopped")
    print("=" * 70)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nNavigation stopped by user")
        print("Goodbye!")
    except Exception as e:
        print(f"\nUnexpected system error: {e}")
        print("Please check your hardware connections and dependencies")
