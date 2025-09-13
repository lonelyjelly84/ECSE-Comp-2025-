#!/usr/bin/env python3
"""
ECSE Robot Arm Face Controller
Integrates face recognition with arm movements
Face Detection → Arm Movement Triggers
"""

import sys
import os
import time

# Add firmware path to access robot controller
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'Firmware'))
from robot_control import ArmRobot, arm_servo1, arm_servo2

# Import LCD display functionality
try:
    from I2C_LCD_DISPLAY import lcd_init, lcd_display_string
    LCD_AVAILABLE = True
    # Initialize LCD
    lcd_init()
except ImportError:
    LCD_AVAILABLE = False
    print("Warning: LCD display not available")
    def lcd_display_string(text, line):
        print(f"[LCD LINE {line}]: {text}")

# Import face recognition system
from face_hello import OpenCVFaceRecognitionSystem


class ArmFaceController:
    def __init__(self):
        """Initialize arm controller and face recognition"""
        self.arm_robot = ArmRobot(arm_servo1, arm_servo2)
        self.face_system = OpenCVFaceRecognitionSystem()
        self.last_action_time = 0
        self.last_face_detected_time = 0  # Start at 0 so robot begins lonely
        self.action_cooldown = 3.0  # Minimum seconds between arm movements
        self.loneliness_threshold = 5.0  # Seconds before feeling lonely
        self.lonely_message_shown = False  # Prevent spam lonely messages
        self.is_currently_lonely = True  # Start in lonely state
        
    def wave_up_down(self, repetitions=3, duration=0.5):
        """Make arm wave up and down motion"""
        print("Waving up and down...")
        # Import the move_servo function
        from robot_control import move_servo
        
        for i in range(repetitions):
            # Move arm up
            move_servo(self.arm_robot.servo1, 90, 45, duration=duration)  # 45 degrees (up)
            # Move arm down  
            move_servo(self.arm_robot.servo1, 45, 135, duration=duration)  # 135 degrees (down)
        # Return to center
        self.arm_robot.center()

    def display_greeting_on_lcd(self, person_name):
        """Display greeting message on LCD"""
        if LCD_AVAILABLE:
            lcd_display_string("Hello", 1)  # Line 1: "Hello"
            lcd_display_string(person_name, 2)  # Line 2: Person's name
        else:
            print(f"[LCD] Hello")
            print(f"[LCD] {person_name}")
    
    def check_loneliness(self):
        """Check if robot has been alone too long and display lonely message"""
        time_since_last_face = time.time() - self.last_face_detected_time
        
        if time_since_last_face >= self.loneliness_threshold and not self.lonely_message_shown:
            print("😢 I'm so lonely...")
            
            # Display on LCD too
            if LCD_AVAILABLE:
                lcd_display_string("I'm so lonely...", 1)
                lcd_display_string("", 2)  # Clear second line
            else:
                print("[LCD] I'm so lonely...")
            
            # Optional: Make a sad arm gesture
            self.sad_arm_gesture()
            
            self.lonely_message_shown = True
            self.is_currently_lonely = True
            print("No faces detected - feeling lonely")
    
    def show_initial_loneliness(self):
        """Show lonely state when robot first starts up"""
        print("😢 Starting up... I'm so lonely...")
        
        if LCD_AVAILABLE:
            lcd_display_string("I'm so lonely...", 1)
            lcd_display_string("Waiting for faces", 2)
        else:
            print("[LCD] I'm so lonely...")
            print("[LCD] Waiting for faces")
        
        # Make initial sad gesture
        self.sad_arm_gesture()
    
    def sad_arm_gesture(self):
        """Make a sad drooping arm gesture when lonely"""
        print("Making sad gesture...")
        from robot_control import move_servo
        
        # Slowly droop the arm down
        move_servo(self.arm_robot.servo1, 90, 150, duration=2.0)  # Droop down slowly
        time.sleep(1)
        move_servo(self.arm_robot.servo1, 150, 90, duration=1.5)  # Slowly back to center
    
    def reset_loneliness(self):
        """Reset loneliness state when a face is detected"""
        was_lonely = self.is_currently_lonely
        
        self.last_face_detected_time = time.time()
        self.lonely_message_shown = False
        self.is_currently_lonely = False
        
        # Only clear LCD if we were actually showing lonely message
        if was_lonely and LCD_AVAILABLE:
            # Don't clear LCD here - let the greeting message override it
            pass
    
    def should_trigger_action(self):
        """Check if enough time has passed since last action"""
        return time.time() - self.last_action_time > self.action_cooldown
    
    def face_detected_greeting(self, person_name):
        """Trigger when any face is detected - greeting sequence"""
        if not self.should_trigger_action():
            return
            
        print(f"🤖 Face detected: {person_name}")
        print("Performing greeting sequence...")
        
        self.arm_robot.raise_arm(duration=1.0)
        time.sleep(1)
        self.arm_robot.wave(repetitions=2)
        time.sleep(1)
        self.arm_robot.lower_arm(duration=1.0)
        
        self.last_action_time = time.time()
        print("Greeting complete!")
    
    def known_person_detected(self, person_name):
        """Trigger when a known person is recognized"""
        if not self.should_trigger_action():
            return
        
        # Reset loneliness since we detected a face
        self.reset_loneliness()
            
        print(f"🎉 Welcome back, {person_name}!")
        
        # Display on LCD: "Hello" and person's name (without .png extension)
        display_name = person_name.replace('.png', '').replace('.jpg', '').replace('.jpeg', '')
        self.display_greeting_on_lcd(display_name)
        
        print("Performing special welcome...")
        
        # Raise arm and wave up and down
        self.arm_robot.raise_arm(duration=0.8)
        time.sleep(0.5)
        self.wave_up_down(repetitions=3)  # Wave up and down 3 times
        time.sleep(0.5)
        self.arm_robot.lower_arm(duration=0.8)
        
        self.last_action_time = time.time()
        print(f"Welcome sequence complete for {display_name}!")
    
    def unknown_person_detected(self):
        """Trigger when unknown person is detected"""
        if not self.should_trigger_action():
            return
        
        # Reset loneliness since we detected a face
        self.reset_loneliness()
            
        print("👋 Unknown person detected")
        print("Performing simple greeting...")
        
        # Simple sequence for unknown people
        self.arm_robot.raise_arm(duration=1.2)
        time.sleep(2)  # Hold position longer
        self.arm_robot.lower_arm(duration=1.2)
        
        self.last_action_time = time.time()
        print("Simple greeting complete!")
    
    def run_continuous_monitoring(self, min_confidence=45):
        """Continuously monitor for faces and trigger arm movements"""
        print("🤖 Starting Arm Face Controller...")
        print("Press Ctrl+C to stop")
        print(f"Known faces: {', '.join(self.face_system.known_face_names) if self.face_system.known_face_names else 'None'}")
        print("-" * 50)
        
        # Show initial lonely state
        self.show_initial_loneliness()
        
        try:
            while True:
                # Look for a face with sufficient confidence
                recognized_name = self.face_system.recognize_single_face(
                    min_confidence=min_confidence, 
                    timeout_seconds=10
                )
                
                if recognized_name:
                    # Known person detected
                    self.known_person_detected(recognized_name)
                else:
                    # Check if any face was detected (even unknown)
                    # For this, we'll do a quick check with lower confidence
                    any_face = self.face_system.recognize_single_face(
                        min_confidence=20,  # Lower threshold for any face
                        timeout_seconds=3
                    )
                    
                    if any_face and "Unknown" in any_face:
                        self.unknown_person_detected()
                
                # Small delay before next scan
                time.sleep(1)
                
                # Check if robot has been lonely too long
                self.check_loneliness()
                
        except KeyboardInterrupt:
            print("\n🛑 Stopping Arm Face Controller...")
            self.cleanup()
    
    def run_single_detection(self, min_confidence=45):
        """Run single face detection and arm movement"""
        print("🤖 Single Detection Mode")
        print("Looking for a face...")
        
        recognized_name = self.face_system.recognize_single_face(
            min_confidence=min_confidence,
            timeout_seconds=30
        )
        
        if recognized_name:
            self.known_person_detected(recognized_name)
            return recognized_name
        else:
            print("No face recognized, trying to detect any face...")
            any_face = self.face_system.recognize_single_face(
                min_confidence=20,
                timeout_seconds=10
            )
            
            if any_face:
                self.unknown_person_detected()
                return "Unknown"
            else:
                print("No face detected")
                return None
    
    def cleanup(self):
        """Clean up resources"""
        print("Returning arm to neutral position...")
        self.arm_robot.stop()
        arm_servo1.stop()
        arm_servo2.stop()
        
        # Import GPIO cleanup
        try:
            import RPi.GPIO as GPIO
            GPIO.cleanup()
        except ImportError:
            pass  # Mock GPIO doesn't need cleanup
        
        print("Cleanup complete!")


def main():
    """Main function with different modes"""
    controller = ArmFaceController()
    
    print("ECSE Arm Face Controller")
    print("=" * 30)
    print("1. Continuous monitoring mode")
    print("2. Single detection mode")
    print("3. Test arm movements only")
    
    try:
        choice = input("Choose mode (1-3): ").strip()
        
        if choice == "1":
            controller.run_continuous_monitoring()
        elif choice == "2":
            result = controller.run_single_detection()
            print(f"Result: {result}")
        elif choice == "3":
            print("Testing arm movements...")
            controller.face_detected_greeting("Test Person")
        else:
            print("Invalid choice")
            
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        controller.cleanup()


if __name__ == "__main__":
    main()
