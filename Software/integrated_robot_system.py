#!/usr/bin/env python3
"""
ECSE Integrated Robot System
Combines face recognition + arm movements + personality + sensors + state machine
"""

import time, math, threading, random, sys, os
try:
    import smbus2 as smbus  # Use smbus2 package but keep smbus interface
except ImportError:
    try:
        import smbus  # Fallback to system smbus if available
    except ImportError:
        # Mock smbus for development
        class MockSMBus:
            def __init__(self, bus):
                self.bus = bus
                print(f"[MOCK] SMBus({bus})")
            
            def write_byte(self, addr, data):
                print(f"[MOCK] write_byte(addr=0x{addr:02X}, data=0x{data:02X})")
        
        smbus = type('smbus', (), {'SMBus': MockSMBus})()

try:
    import pygame
except ImportError:
    # Mock pygame for development
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
except ImportError:
    # Basic numpy mock for development
    class MockNumpy:
        @staticmethod
        def array(data): return data
    np = MockNumpy()

# Add firmware path to access robot controller
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'Firmware'))
from robot_control import ArmRobot, arm_servo1, arm_servo2, move_servo

# Import face recognition system
from face_hello import OpenCVFaceRecognitionSystem

# LCD and I2C setup
channel = 1
bus = smbus.SMBus(1)
module_address = 0x27

# Sensor GPIO
TRIG_PIN = 17
ECHO_PIN = 27

# LCD Constants
LCD_CHR = 1
LCD_CMD = 0
LCD_LINE_1 = 0x80
LCD_LINE_2 = 0xC0
LCD_LINE_3 = 0x94
LCD_LINE_4 = 0xD4

# Timing Constants
E_PULSE = 0.0005
E_DELAY = 0.0005

# Backlight
LCD_BACKLIGHT = 0x08

# GPIO setup with lgpio (optimized for Pi 4)
try:
    import lgpio
    # Pi 4 has gpiochip0 as the main chip
    gpio_chip = lgpio.gpiochip_open(0)
    GPIO_AVAILABLE = True
    print("[INFO] lgpio initialized for Raspberry Pi 4")
except ImportError:
    # Mock lgpio for development
    class MockLGPIO:
        def gpiochip_open(self, chip):
            print(f"[MOCK] gpiochip_open({chip}) - Pi 4 simulation")
            return 0
            
        def gpiochip_close(self, handle):
            print(f"[MOCK] gpiochip_close({handle})")
            
        def gpio_claim_output(self, handle, gpio, level=0):
            print(f"[MOCK] gpio_claim_output(handle={handle}, gpio={gpio}, level={level})")
            
        def gpio_claim_input(self, handle, gpio):
            print(f"[MOCK] gpio_claim_input(handle={handle}, gpio={gpio})")
            
        def gpio_write(self, handle, gpio, level):
            print(f"[MOCK] gpio_write(handle={handle}, gpio={gpio}, level={level})")
            
        def gpio_read(self, handle, gpio):
            print(f"[MOCK] gpio_read(handle={handle}, gpio={gpio}) -> 0")
            return 0
            
        def gpio_free(self, handle, gpio):
            print(f"[MOCK] gpio_free(handle={handle}, gpio={gpio})")
    
    lgpio = MockLGPIO()
    gpio_chip = lgpio.gpiochip_open(0)
    GPIO_AVAILABLE = False
    print("[MOCK] lgpio mock initialized for development")


# LCD Functions
def lcd_toggle_enable(data):
    try:
        bus.write_byte(module_address, data | 0x04)
        time.sleep(E_PULSE)
        bus.write_byte(module_address, data & ~0x04)
        time.sleep(E_DELAY)
    except:
        print("[LCD] Toggle enable failed")

def lcd_send_byte(bits, mode):
    try:
        # Send high nibble
        high = mode | (bits & 0xF0) | LCD_BACKLIGHT
        bus.write_byte(module_address, high)
        lcd_toggle_enable(high)
        
        low = mode | ((bits << 4) & 0xF8) | LCD_BACKLIGHT
        bus.write_byte(module_address, low)
        lcd_toggle_enable(low)
    except:
        print(f"[LCD] Send byte failed: {bits}")

def lcd_init():
    """Initialize the LCD display"""
    try:
        lcd_send_byte(0x33, LCD_CMD)
        lcd_send_byte(0x32, LCD_CMD)
        lcd_send_byte(0x06, LCD_CMD)
        lcd_send_byte(0x0C, LCD_CMD)
        lcd_send_byte(0x28, LCD_CMD)
        lcd_send_byte(0x01, LCD_CMD)
        time.sleep(E_DELAY)
        print("LCD initialized successfully")
        return True
    except:
        print("LCD initialization failed - using fallback")
        return False

def lcd_display_string(message, line):
    """Send string to display"""
    try:
        message = message.ljust(16, " ")
        if line == 1:
            lcd_send_byte(LCD_LINE_1, LCD_CMD)
        elif line == 2:
            lcd_send_byte(LCD_LINE_2, LCD_CMD)
        elif line == 3:
            lcd_send_byte(LCD_LINE_3, LCD_CMD)
        elif line == 4:
            lcd_send_byte(LCD_LINE_4, LCD_CMD)
            
        for char in message:
            lcd_send_byte(ord(char), LCD_CHR)
    except:
        print(f"[LCD FALLBACK] Line {line}: {message}")


# Sensor Functions
def sense_setup():
    """Setup ultrasonic sensor"""
    try:
        if GPIO_AVAILABLE:
            # Claim pins for output (TRIG) and input (ECHO)
            lgpio.gpio_claim_output(gpio_chip, TRIG_PIN, 0)  # Start low
            lgpio.gpio_claim_input(gpio_chip, ECHO_PIN)
            time.sleep(0.1)
            print("Ultrasonic sensor initialized")
            return True
        else:
            print("GPIO not available - using mock sensor")
            return False
    except Exception as e:
        print(f"Sensor setup failed: {e} - using mock")
        return False

def get_distance():
    """Get distance from ultrasonic sensor (optimized for Pi 4)"""
    try:
        if GPIO_AVAILABLE:
            # Send trigger pulse
            lgpio.gpio_write(gpio_chip, TRIG_PIN, 1)
            time.sleep(0.00001)  # 10μs pulse
            lgpio.gpio_write(gpio_chip, TRIG_PIN, 0)
            
            # Wait for echo start with timeout (Pi 4 optimized)
            timeout = time.time() + 0.1  # 100ms timeout
            start_time = time.time()
            while lgpio.gpio_read(gpio_chip, ECHO_PIN) == 0:
                start_time = time.time()
                if time.time() > timeout:
                    return -1  # Timeout error
                    
            # Wait for echo end with timeout
            timeout = time.time() + 0.1  # 100ms timeout
            end_time = start_time
            while lgpio.gpio_read(gpio_chip, ECHO_PIN) == 1:
                end_time = time.time()
                if time.time() > timeout:
                    return -1  # Timeout error
                    
            elapsed = end_time - start_time
            # Speed of sound at 20°C: 343 m/s = 34300 cm/s
            distance = (elapsed * 34300) / 2
            
            # Validate reasonable distance range (2cm to 400cm for HC-SR04)
            if 2 <= distance <= 400:
                return distance
            else:
                return -1  # Out of range
        else:
            # Mock distance for testing
            return random.uniform(20, 100)
    except Exception as e:
        print(f"[SENSOR] Error reading distance: {e}")
        # Mock distance for testing
        return random.uniform(20, 100)

def am_u_too_close():
    """Check if someone is too close (within 30cm)"""
    distances = []
    for i in range(4):
        ret = get_distance()
        if ret > -1:
            distances.append(ret)
            
    if len(distances) == 0:
        return False
    else:
        avg_distance = sum(distances) / len(distances)
        print(f"Average distance: {avg_distance:.1f}cm")
        return avg_distance < 30


# Audio Functions
def play_mp3(filename):
    """Play MP3 file"""
    try:
        pygame.mixer.init()
        pygame.mixer.music.load(filename)
        pygame.mixer.music.play()
        print(f"Playing: {filename}")
    except:
        print(f"[AUDIO] Would play: {filename}")


class IntegratedRobotSystem:
    def __init__(self):
        """Initialize the complete robot system"""
        # Initialize subsystems
        self.lcd_available = lcd_init()
        self.sensor_available = sense_setup()
        
        # Initialize face recognition and arm control
        self.arm_robot = ArmRobot(arm_servo1, arm_servo2)
        self.face_system = OpenCVFaceRecognitionSystem()
        
        # State management
        self.current_state = "IDLE"
        self.last_face_time = 0
        self.last_action_time = 0
        self.action_cooldown = 3.0
        
        # Audio setup
        try:
            pygame.mixer.init()
        except:
            print("Audio system not available")
    
    def display_message(self, line1, line2=""):
        """Display message on LCD with fallback"""
        lcd_display_string(line1, 1)
        if line2:
            lcd_display_string(line2, 2)
    
    def should_act(self):
        """Check if enough time passed for new action"""
        return time.time() - self.last_action_time > self.action_cooldown
    
    # STATE: IDLE (Lonely)
    def state_idle(self):
        """Idle state - robot is lonely and waiting"""
        print("🤖 STATE: IDLE (Lonely)")
        
        # Play lonely music
        if not pygame.mixer.music.get_busy():
            play_mp3("/home/josh/Downloads/spamton dance.mp3")
        
        # Display lonely message
        self.display_message("I am so Alone", "Looking for friends")
        
        # Sad arm gesture
        if self.should_act():
            self.sad_arm_gesture()
            self.last_action_time = time.time()
        
        time.sleep(2)
        
        # Check for transitions
        return self.check_state_transitions()
    
    # STATE: FRIEND DETECTED
    def state_friend_detected(self, person_name=None):
        """Friend detected state - robot is happy"""
        print(f"🎉 STATE: FRIEND DETECTED - {person_name or 'Unknown Friend'}")
        
        # Play happy music
        if not pygame.mixer.music.get_busy():
            play_mp3("/home/josh/Downloads/HEY EVERY !.mp3")
        
        if person_name and person_name != "Unknown":
            # Known person
            clean_name = person_name.replace('.png', '').replace('.jpg', '').replace('.jpeg', '')
            self.display_message("Hey Hey Hey!", f"Hi {clean_name}!")
            
            # Enthusiastic greeting
            if self.should_act():
                self.enthusiastic_greeting()
                self.last_action_time = time.time()
                
        else:
            # Unknown person
            self.display_message("Hey Hey Hey!", "New friend detected!")
            
            # Simple greeting
            if self.should_act():
                self.simple_greeting()
                self.last_action_time = time.time()
        
        # Stay happy while person is close
        start_time = time.time()
        while time.time() - start_time < 10:  # Stay in friend mode for 10 seconds
            if am_u_too_close():
                self.display_message("I love you!", "<3")
                time.sleep(2)
                self.display_message("Good feeling!", "Stay close friend!")
                time.sleep(2)
            else:
                break
        
        return self.check_state_transitions()
    
    # STATE: TOO CLOSE
    def state_too_close(self):
        """Someone is too close - warning state"""
        print("⚠️ STATE: TOO CLOSE")
        
        self.display_message("Whoa there!", "Too close buddy!")
        
        # Defensive arm gesture
        if self.should_act():
            self.defensive_gesture()
            self.last_action_time = time.time()
        
        time.sleep(1)
        return self.check_state_transitions()
    
    def check_state_transitions(self):
        """Check what state to transition to next"""
        # Check if someone is too close
        if am_u_too_close():
            if self.current_state != "TOO_CLOSE":
                self.current_state = "TOO_CLOSE"
                return "TOO_CLOSE"
        
        # Check for face detection
        recognized_name = self.face_system.recognize_single_face(
            min_confidence=45, 
            timeout_seconds=3
        )
        
        if recognized_name:
            self.last_face_time = time.time()
            if self.current_state != "FRIEND_DETECTED":
                self.current_state = "FRIEND_DETECTED"
                return ("FRIEND_DETECTED", recognized_name)
        
        # Check if we should go back to idle
        time_since_face = time.time() - self.last_face_time
        if time_since_face > 5 and not am_u_too_close():
            if self.current_state != "IDLE":
                self.current_state = "IDLE"
                return "IDLE"
        
        return self.current_state
    
    # ARM GESTURES
    def sad_arm_gesture(self):
        """Sad drooping gesture"""
        print("😢 Making sad gesture...")
        move_servo(self.arm_robot.servo1, 90, 150, duration=2.0)  # Droop down
        time.sleep(1)
        move_servo(self.arm_robot.servo1, 150, 90, duration=1.5)  # Back to center
    
    def enthusiastic_greeting(self):
        """Happy enthusiastic greeting for known people"""
        print("🎉 Enthusiastic greeting!")
        self.arm_robot.raise_arm(duration=0.8)
        time.sleep(0.5)
        
        # Wave up and down enthusiastically
        for _ in range(3):
            move_servo(self.arm_robot.servo1, 90, 45, duration=0.3)  # Up
            move_servo(self.arm_robot.servo1, 45, 135, duration=0.3)  # Down
        
        self.arm_robot.center()
    
    def simple_greeting(self):
        """Simple greeting for unknown people"""
        print("👋 Simple greeting")
        self.arm_robot.raise_arm(duration=1.0)
        time.sleep(2)
        self.arm_robot.lower_arm(duration=1.0)
    
    def defensive_gesture(self):
        """Defensive gesture when too close"""
        print("🛡️ Defensive gesture")
        move_servo(self.arm_robot.servo1, 90, 60, duration=0.5)  # Slight back
        time.sleep(1)
        self.arm_robot.center()
    
    def run_main_loop(self):
        """Main state machine loop"""
        print("🤖 Starting Integrated Robot System")
        print("=" * 50)
        print("States: IDLE → FRIEND_DETECTED ↔ TOO_CLOSE")
        print("Press Ctrl+C to stop")
        print("=" * 50)
        
        # Initialize in idle state
        self.current_state = "IDLE"
        
        try:
            while True:
                if self.current_state == "IDLE":
                    next_state = self.state_idle()
                    
                elif self.current_state == "FRIEND_DETECTED":
                    # Need to get the person name for friend state
                    next_state = self.state_friend_detected()
                    
                elif self.current_state == "TOO_CLOSE":
                    next_state = self.state_too_close()
                
                # Handle state transitions
                if isinstance(next_state, tuple):
                    # Friend detected with name
                    self.current_state = next_state[0]
                    if len(next_state) > 1:
                        # Re-run friend state with the detected name
                        self.state_friend_detected(next_state[1])
                elif isinstance(next_state, str):
                    self.current_state = next_state
                
                time.sleep(0.5)  # Small delay between state checks
                
        except KeyboardInterrupt:
            print("\n🛑 Shutting down robot...")
            self.cleanup()
    
    def cleanup(self):
        """Clean shutdown"""
        print("Returning to neutral position...")
        self.arm_robot.stop()
        
        try:
            pygame.mixer.music.stop()
            arm_servo1.stop()
            arm_servo2.stop()
            if GPIO_AVAILABLE:
                # Free GPIO pins
                lgpio.gpio_free(gpio_chip, TRIG_PIN)
                lgpio.gpio_free(gpio_chip, ECHO_PIN)
                lgpio.gpiochip_close(gpio_chip)
        except:
            pass
        
        print("Cleanup complete!")


def main():
    """Main function"""
    robot = IntegratedRobotSystem()
    robot.run_main_loop()


if __name__ == "__main__":
    main()
