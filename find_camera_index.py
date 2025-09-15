#!/usr/bin/env python3
"""
Camera Index Finder for Pi 3
Finds the correct camera index when getting "camera index out of range" error
"""

import cv2
import sys

def find_working_camera():
    """Find the first working camera index"""
    print("🔍 Searching for working camera...")
    print("This will test camera indices 0-10")
    print("")
    
    working_cameras = []
    
    for i in range(11):  # Test indices 0-10
        print(f"Testing camera index {i}...", end=" ")
        
        try:
            cap = cv2.VideoCapture(i)
            
            if cap.isOpened():
                ret, frame = cap.read()
                if ret and frame is not None:
                    height, width = frame.shape[:2]
                    print(f"✅ WORKING - Resolution: {width}x{height}")
                    working_cameras.append(i)
                else:
                    print("⚠️  Opens but no frame")
                cap.release()
            else:
                print("❌ Not available")
                
        except Exception as e:
            print(f"❌ Error: {e}")
            
    print("")
    print("=" * 50)
    print("RESULTS:")
    print("=" * 50)
    
    if working_cameras:
        print(f"✅ Working camera indices: {working_cameras}")
        print(f"🎯 Recommended camera index: {working_cameras[0]}")
        return working_cameras[0]
    else:
        print("❌ No working cameras found!")
        print("")
        print("Troubleshooting tips:")
        print("1. Check camera cable connection")
        print("2. Enable camera in raspi-config:")
        print("   sudo raspi-config")
        print("   → Interface Options → Camera → Enable")
        print("3. Reboot after enabling camera")
        print("4. Check if camera is detected:")
        print("   vcgencmd get_camera")
        print("5. For USB cameras, check:")
        print("   lsusb")
        return None

def test_camera_thoroughly(camera_index):
    """Test the camera thoroughly"""
    print(f"")
    print(f"🧪 Testing camera {camera_index} thoroughly...")
    print("=" * 50)
    
    try:
        cap = cv2.VideoCapture(camera_index)
        
        if not cap.isOpened():
            print(f"❌ Camera {camera_index} failed to open")
            return False
            
        # Test basic capture
        print("📸 Testing basic capture...")
        ret, frame = cap.read()
        if ret:
            print(f"✅ Frame captured: {frame.shape}")
        else:
            print("❌ Failed to capture frame")
            cap.release()
            return False
            
        # Test camera properties
        print("📊 Camera properties:")
        width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        fps = cap.get(cv2.CAP_PROP_FPS)
        print(f"   Resolution: {int(width)} x {int(height)}")
        print(f"   FPS: {fps}")
        
        # Test continuous capture
        print("🎬 Testing continuous capture (5 seconds)...")
        start_time = cv2.getTickCount()
        frame_count = 0
        
        for _ in range(50):  # About 5 seconds at 10 FPS
            ret, frame = cap.read()
            if ret:
                frame_count += 1
            else:
                print(f"❌ Frame {frame_count + 1} failed")
                break
                
        end_time = cv2.getTickCount()
        elapsed = (end_time - start_time) / cv2.getTickFrequency()
        actual_fps = frame_count / elapsed
        
        print(f"✅ Captured {frame_count} frames in {elapsed:.1f} seconds")
        print(f"📈 Actual FPS: {actual_fps:.1f}")
        
        cap.release()
        return True
        
    except Exception as e:
        print(f"❌ Camera test error: {e}")
        return False

def create_fixed_robot_code(camera_index):
    """Create a fixed version of the robot code with correct camera index"""
    print(f"")
    print(f"🔧 Creating fixed robot code for camera index {camera_index}...")
    
    # Read the current robot_name.py
    try:
        with open('robot_name.py', 'r') as f:
            content = f.read()
            
        # Replace the camera index
        old_line = "video_capture = cv2.VideoCapture(0)"
        new_line = f"video_capture = cv2.VideoCapture({camera_index})"
        
        if old_line in content:
            fixed_content = content.replace(old_line, new_line)
            
            # Write to a backup file first
            with open('robot_name_fixed.py', 'w') as f:
                f.write(fixed_content)
                
            print(f"✅ Created robot_name_fixed.py with camera index {camera_index}")
            print("To use the fix:")
            print("  mv robot_name.py robot_name_backup.py")
            print("  mv robot_name_fixed.py robot_name.py")
            
        else:
            print("⚠️  Could not find camera initialization line to fix")
            
    except Exception as e:
        print(f"❌ Error creating fixed code: {e}")

def main():
    print("🤖 Pi 3 Camera Index Finder")
    print("Solving 'camera index out of range' error")
    print("=" * 50)
    
    # Find working camera
    working_index = find_working_camera()
    
    if working_index is not None:
        print("")
        print("🎯 SOLUTION FOUND!")
        print(f"Use camera index {working_index} in your robot code")
        
        # Test the camera thoroughly
        if test_camera_thoroughly(working_index):
            print("")
            print("✅ Camera is working properly!")
            
            # Offer to create fixed code
            response = input("\nCreate fixed robot code? (y/n): ").lower().strip()
            if response == 'y':
                create_fixed_robot_code(working_index)
        else:
            print("❌ Camera has issues - check hardware connection")
    else:
        print("")
        print("❌ NO SOLUTION - No working cameras found")
        print("Check hardware connections and camera enable status")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n🛑 Cancelled by user")
    except Exception as e:
        print(f"\n💥 Error: {e}")