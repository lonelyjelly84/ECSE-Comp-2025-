#!/usr/bin/env python3
"""
Camera Test Script for Raspberry Pi
Tests camera functionality step by step for the ECSE robot project
"""

import cv2
import time
import sys

def test_camera_basic():
    """Basic camera connection test"""
    print("=" * 50)
    print("🔍 TEST 1: Basic Camera Connection")
    print("=" * 50)
    
    try:
        cap = cv2.VideoCapture(0)
        ret, frame = cap.read()
        
        if ret:
            height, width = frame.shape[:2]
            print(f"✅ Camera OK - Resolution: {width}x{height}")
            print(f"   Frame shape: {frame.shape}")
        else:
            print("❌ Camera FAILED - Could not read frame")
            
        cap.release()
        return ret
        
    except Exception as e:
        print(f"❌ Camera ERROR: {e}")
        return False

def test_camera_properties():
    """Test camera properties and settings"""
    print("\n" + "=" * 50)
    print("📊 TEST 2: Camera Properties")
    print("=" * 50)
    
    try:
        cap = cv2.VideoCapture(0)
        
        if not cap.isOpened():
            print("❌ Camera failed to open")
            return False
            
        # Get camera properties
        width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        fps = cap.get(cv2.CAP_PROP_FPS)
        
        print(f"📐 Resolution: {int(width)} x {int(height)}")
        print(f"🎬 FPS: {fps}")
        print(f"🔧 Backend: {cap.getBackendName()}")
        
        cap.release()
        return True
        
    except Exception as e:
        print(f"❌ Properties ERROR: {e}")
        return False

def test_camera_capture():
    """Test continuous frame capture"""
    print("\n" + "=" * 50)
    print("📹 TEST 3: Continuous Capture (10 seconds)")
    print("=" * 50)
    
    try:
        cap = cv2.VideoCapture(0)
        
        if not cap.isOpened():
            print("❌ Camera failed to open")
            return False
            
        print("📹 Capturing frames for 10 seconds...")
        start_time = time.time()
        frame_count = 0
        
        while time.time() - start_time < 10:
            ret, frame = cap.read()
            if ret:
                frame_count += 1
            else:
                print(f"❌ Frame {frame_count + 1} failed")
                break
            
            # Print progress every 2 seconds
            elapsed = time.time() - start_time
            if frame_count % 60 == 0:  # Assuming ~30 FPS
                print(f"   📊 {elapsed:.1f}s - {frame_count} frames captured")
                
        cap.release()
        
        avg_fps = frame_count / 10
        print(f"✅ Captured {frame_count} frames in 10 seconds")
        print(f"📈 Average FPS: {avg_fps:.1f}")
        
        return frame_count > 0
        
    except Exception as e:
        print(f"❌ Capture ERROR: {e}")
        return False

def test_face_detection():
    """Test OpenCV face detection with known/unknown face recognition"""
    print("\n" + "=" * 50)
    print("👤 TEST 4: Face Detection + Recognition (20 seconds)")
    print("=" * 50)
    print("🎯 Move your face in front of the camera...")
    print("📋 Testing both known and unknown face detection...")
    
    try:
        # Load face cascade
        face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
        
        if face_cascade.empty():
            print("❌ Could not load face cascade classifier")
            return False
        
        # Try to load face recognition system
        face_recognizer = None
        known_names = []
        
        try:
            import pickle
            import os
            
            # Check if face recognition data exists
            if os.path.exists("opencv_face_data.pkl") and os.path.exists("opencv_face_data_model.yml"):
                print("📚 Loading face recognition data...")
                
                # Load names
                with open("opencv_face_data.pkl", 'rb') as f:
                    data = pickle.load(f)
                    known_names = data['names']
                
                # Load recognizer
                face_recognizer = cv2.face.LBPHFaceRecognizer_create()
                face_recognizer.read("opencv_face_data_model.yml")
                
                print(f"✅ Loaded faces: {', '.join(known_names)}")
            else:
                print("⚠️  No face recognition data found - will only detect unknown faces")
                print("   (This is normal if you haven't trained the system yet)")
                
        except Exception as e:
            print(f"⚠️  Face recognition loading failed: {e}")
            print("   Will only detect faces as 'Unknown'")
            face_recognizer = None
            
        cap = cv2.VideoCapture(0)
        
        if not cap.isOpened():
            print("❌ Camera failed to open")
            return False
            
        start_time = time.time()
        total_faces = 0
        face_detections = 0
        known_face_detections = 0
        unknown_face_detections = 0
        recognized_names = set()
        
        while time.time() - start_time < 20:
            ret, frame = cap.read()
            if not ret:
                print("❌ Failed to read frame")
                break
                
            # Convert to grayscale for face detection
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Detect faces
            faces = face_cascade.detectMultiScale(gray, 1.3, 5, minSize=(50, 50))
            
            if len(faces) > 0:
                face_detections += 1
                total_faces += len(faces)
                
                face_info = []
                
                # Try to recognize each face
                for (x, y, w, h) in faces:
                    if face_recognizer is not None and known_names:
                        # Extract face for recognition
                        face_roi = gray[y:y+h, x:x+w]
                        face_roi = cv2.resize(face_roi, (200, 200))
                        
                        # Predict
                        label, confidence = face_recognizer.predict(face_roi)
                        
                        # Lower confidence = better match (typical threshold: 100)
                        if confidence < 100 and label < len(known_names):
                            name = known_names[label]
                            face_info.append(f"👤 {name} (confidence: {confidence:.1f})")
                            recognized_names.add(name)
                            known_face_detections += 1
                        else:
                            face_info.append(f"❓ Unknown (confidence: {confidence:.1f})")
                            unknown_face_detections += 1
                    else:
                        face_info.append("❓ Unknown (no training data)")
                        unknown_face_detections += 1
                
                print(f"👥 Faces detected: {', '.join(face_info)} (Total detections: {face_detections})")
                
            time.sleep(0.2)  # 5 FPS for face detection + recognition
            
        cap.release()
        
        print(f"✅ Face detection + recognition test complete!")
        print(f"📊 Total face detections: {face_detections}")
        print(f"👥 Total faces seen: {total_faces}")
        print(f"👤 Known faces detected: {known_face_detections}")
        print(f"❓ Unknown faces detected: {unknown_face_detections}")
        
        if recognized_names:
            print(f"🎯 Recognized people: {', '.join(recognized_names)}")
        else:
            print("🔍 No known faces recognized (add your face to known_faces folder!)")
        
        return face_detections > 0
        
    except Exception as e:
        print(f"❌ Face detection ERROR: {e}")
        return False

def test_camera_devices():
    """Check available camera devices"""
    print("\n" + "=" * 50)
    print("🔍 TEST 5: Available Camera Devices")
    print("=" * 50)
    
    available_cameras = []
    
    for i in range(5):  # Check first 5 camera indices
        try:
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                ret, frame = cap.read()
                if ret:
                    print(f"✅ Camera {i}: Working")
                    available_cameras.append(i)
                else:
                    print(f"⚠️  Camera {i}: Opens but no frame")
                cap.release()
            else:
                print(f"❌ Camera {i}: Not available")
        except:
            print(f"❌ Camera {i}: Error")
            
    print(f"📊 Available cameras: {available_cameras}")
    return len(available_cameras) > 0

def main():
    """Run all camera tests"""
    print("🤖 ECSE Robot Camera Test Suite")
    print("🔧 Testing camera for face recognition system")
    print("⏰ Started:", time.strftime("%Y-%m-%d %H:%M:%S"))
    
    # Run all tests
    tests = [
        ("Basic Connection", test_camera_basic),
        ("Camera Properties", test_camera_properties), 
        ("Continuous Capture", test_camera_capture),
        ("Face Detection + Recognition", test_face_detection),
        ("Camera Devices", test_camera_devices)
    ]
    
    results = []
    
    for test_name, test_func in tests:
        try:
            result = test_func()
            results.append((test_name, result))
        except Exception as e:
            print(f"❌ {test_name} CRASHED: {e}")
            results.append((test_name, False))
    
    # Summary
    print("\n" + "=" * 50)
    print("📋 TEST SUMMARY")
    print("=" * 50)
    
    passed = 0
    for test_name, result in results:
        status = "✅ PASS" if result else "❌ FAIL"
        print(f"{status} - {test_name}")
        if result:
            passed += 1
    
    print(f"\n🎯 Overall: {passed}/{len(results)} tests passed")
    
    if passed == len(results):
        print("🎉 All tests passed! Camera is ready for the robot system!")
    elif passed >= 3:
        print("⚠️  Most tests passed. Camera should work but check failed tests.")
    else:
        print("❌ Multiple failures. Check camera connection and configuration.")
        
    print("⏰ Completed:", time.strftime("%Y-%m-%d %H:%M:%S"))

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n🛑 Test interrupted by user")
    except Exception as e:
        print(f"\n💥 Unexpected error: {e}")
        sys.exit(1)