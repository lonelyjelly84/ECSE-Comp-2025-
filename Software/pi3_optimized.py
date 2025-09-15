#!/usr/bin/env python3
"""
Pi 3 Optimized Robot System
Performance-tuned version for Raspberry Pi 3 64-bit
"""

import time, math, threading, random, sys, os
import platform

# Detect if running on Pi 3 for performance optimizations
def is_pi3():
    try:
        with open('/proc/device-tree/model', 'r') as f:
            model = f.read()
            return 'Raspberry Pi 3' in model
    except:
        # Fallback: check CPU info
        try:
            with open('/proc/cpuinfo', 'r') as f:
                cpuinfo = f.read()
                return 'ARMv8' in cpuinfo and 'BCM2837' in cpuinfo
        except:
            return False

PI3_DETECTED = is_pi3()
print(f"Pi 3 optimization: {'ENABLED' if PI3_DETECTED else 'DISABLED'}")

# Pi 3 Performance Settings
if PI3_DETECTED:
    # Reduced settings for Pi 3
    CAMERA_WIDTH = 320
    CAMERA_HEIGHT = 240
    CAMERA_FPS = 10
    FACE_DETECTION_INTERVAL = 0.5  # Check every 500ms instead of continuous
    FACE_DETECTION_SCALE = 1.3  # Faster but less accurate
    FACE_MIN_NEIGHBORS = 2  # Reduce for speed
    PROCESSING_DELAY = 0.1  # Add delays to prevent CPU overload
else:
    # Standard settings for Pi 4
    CAMERA_WIDTH = 640
    CAMERA_HEIGHT = 480
    CAMERA_FPS = 15
    FACE_DETECTION_INTERVAL = 0.1
    FACE_DETECTION_SCALE = 1.1
    FACE_MIN_NEIGHBORS = 3
    PROCESSING_DELAY = 0.05

print(f"Camera settings: {CAMERA_WIDTH}x{CAMERA_HEIGHT} @ {CAMERA_FPS}fps")
print(f"Face detection interval: {FACE_DETECTION_INTERVAL}s")

# Import packages with Pi 3 optimizations
try:
    import smbus2 as smbus
except ImportError:
    try:
        import smbus
    except ImportError:
        class MockSMBus:
            def __init__(self, bus):
                self.bus = bus
                print(f"[MOCK] SMBus({bus})")
            def write_byte(self, addr, data):
                print(f"[MOCK] write_byte(addr=0x{addr:02X}, data=0x{data:02X})")
        smbus = type('smbus', (), {'SMBus': MockSMBus})()

try:
    import pygame
    pygame.mixer.init(frequency=22050, size=-16, channels=2, buffer=1024)  # Reduced quality for Pi 3
    print("✓ Pygame loaded (Pi 3 optimized)")
except ImportError:
    class MockPygame:
        class mixer:
            class music:
                @staticmethod
                def load(file): print(f"[MOCK] pygame.mixer.music.load({file})")
                @staticmethod
                def play(loops=0): print(f"[MOCK] pygame.mixer.music.play(loops={loops})")
                @staticmethod
                def stop(): print(f"[MOCK] pygame.mixer.music.stop()")
                @staticmethod
                def get_busy(): return False
            @staticmethod
            def init(): print("[MOCK] pygame.mixer.init()")
    pygame = MockPygame()

try:
    import numpy as np
    print(f"✓ NumPy {np.__version__} loaded")
except ImportError:
    class MockNumpy:
        @staticmethod
        def array(data): return data
    np = MockNumpy()

try:
    import cv2
    print(f"✓ OpenCV {cv2.__version__} loaded")
except ImportError:
    print("⚠ OpenCV not available")
    cv2 = None

# Pi 3 optimized face detection
class Pi3FaceDetector:
    def __init__(self):
        self.last_detection_time = 0
        self.last_result = False
        self.face_cascade = None
        self.camera_init_time = 0
        
        if cv2:
            self.face_cascade = cv2.CascadeClassifier(
                cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
            )
    
    def detect_face_optimized(self):
        """
        Pi 3 optimized face detection with caching and reduced frequency
        """
        current_time = time.time()
        
        # Use cached result if called too frequently
        if current_time - self.last_detection_time < FACE_DETECTION_INTERVAL:
            return self.last_result
        
        try:
            # Limit camera access frequency on Pi 3
            if current_time - self.camera_init_time < 0.2:
                return self.last_result
                
            video_capture = cv2.VideoCapture(0)
            if not video_capture.isOpened():
                print("Camera not available")
                return False
            
            # Pi 3 optimized camera settings
            video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
            video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
            video_capture.set(cv2.CAP_PROP_FPS, CAMERA_FPS)
            video_capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            
            # Allow extra time for Pi 3 camera initialization
            if PI3_DETECTED:
                time.sleep(0.15)
            
            # Capture frame
            ret, frame = video_capture.read()
            video_capture.release()
            self.camera_init_time = time.time()
            
            if not ret or frame is None:
                return False
            
            # Aggressive downscaling for Pi 3
            if PI3_DETECTED:
                small_frame = cv2.resize(frame, (120, 90))  # Even smaller for Pi 3
            else:
                small_frame = cv2.resize(frame, (160, 120))
            
            gray = cv2.cvtColor(small_frame, cv2.COLOR_BGR2GRAY)
            
            # Pi 3 optimized detection parameters
            faces = self.face_cascade.detectMultiScale(
                gray,
                scaleFactor=FACE_DETECTION_SCALE,
                minNeighbors=FACE_MIN_NEIGHBORS,
                minSize=(15, 15) if PI3_DETECTED else (20, 20),
                flags=cv2.CASCADE_SCALE_IMAGE
            )
            
            face_detected = len(faces) > 0
            
            # Cache results
            self.last_detection_time = current_time
            self.last_result = face_detected
            
            if face_detected:
                print(f"✓ Face detected! Found {len(faces)} face(s) [Pi3: {PI3_DETECTED}]")
            else:
                print("✗ No face detected")
            
            # Add processing delay for Pi 3
            if PI3_DETECTED:
                time.sleep(PROCESSING_DELAY)
            
            return face_detected
            
        except Exception as e:
            print(f"Error in face detection: {e}")
            return False

# Global face detector instance
face_detector = Pi3FaceDetector()

def detect_face_quick():
    """
    Wrapper function for compatibility with existing code
    """
    if cv2 is None:
        print("[MOCK] Face detection - returning False")
        return False
    return face_detector.detect_face_optimized()

# Pi 3 System Monitor
class Pi3SystemMonitor:
    @staticmethod
    def get_cpu_temp():
        try:
            with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                temp = int(f.read()) / 1000.0
                return temp
        except:
            return None
    
    @staticmethod
    def get_cpu_usage():
        try:
            with open('/proc/loadavg', 'r') as f:
                load = float(f.read().split()[0])
                return load
        except:
            return None
    
    @staticmethod
    def monitor_system():
        temp = Pi3SystemMonitor.get_cpu_temp()
        load = Pi3SystemMonitor.get_cpu_usage()
        
        if temp and temp > 70:
            print(f"⚠ Pi 3 temperature high: {temp:.1f}°C")
        if load and load > 2:
            print(f"⚠ Pi 3 CPU load high: {load:.2f}")
        
        return temp, load

# Export key functions for robot system
__all__ = [
    'detect_face_quick', 
    'Pi3SystemMonitor',
    'PI3_DETECTED',
    'CAMERA_WIDTH',
    'CAMERA_HEIGHT',
    'PROCESSING_DELAY'
]

if __name__ == "__main__":
    print("Pi 3 Robot System Optimizations")
    print(f"Platform: {platform.platform()}")
    print(f"Pi 3 detected: {PI3_DETECTED}")
    
    # Test face detection
    if cv2:
        print("Testing face detection...")
        for i in range(3):
            result = detect_face_quick()
            print(f"Test {i+1}: {'Face detected' if result else 'No face'}")
            time.sleep(1)
    
    # Monitor system
    temp, load = Pi3SystemMonitor.monitor_system()
    if temp:
        print(f"CPU Temperature: {temp:.1f}°C")
    if load:
        print(f"CPU Load: {load:.2f}")