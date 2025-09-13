import time, math, threading, random
import cv2
import sys
import os

# Add rpi_packages to Python path for local package installation
rpi_packages_path = os.path.join(os.path.dirname(__file__), 'rpi_packages')
if os.path.exists(rpi_packages_path):
    sys.path.insert(0, rpi_packages_path)

# Import packages with fallbacks
try:
    import numpy as np
    print("✓ NumPy loaded")
except ImportError:
    print("⚠ NumPy not available")
    np = None

try:
    import pygame
    pygame.mixer.init()
    print("✓ Pygame loaded")
except ImportError:
    print("⚠ Pygame not available - audio disabled")
    pygame = None

try:
    import smbus
    print("✓ SMBus loaded")
except ImportError:
    try:
        import smbus2 as smbus
        print("✓ SMBus2 loaded")
    except ImportError:
        print("⚠ SMBus not available - LCD disabled")
        smbus = None

# Add Software folder to path for imports
sys.path.append(os.path.join(os.path.dirname(__file__), 'Software'))
sys.path.append(os.path.join(os.path.dirname(__file__), 'Firmware'))

try:
    from face_hello import OpenCVFaceRecognitionSystem
    FACE_DETECTION_AVAILABLE = True
except ImportError:
    print("Warning: Face detection not available - face_hello module not found")
    FACE_DETECTION_AVAILABLE = False

try:
    import robot_control, integrated_robot_system
except ImportError:
    print("Warning: Some robot modules not available")

# Global face recognition system - initialize later
face_recognition_system = None

"""I2C LCD Display Module, could communicat with an Arduino which can then communicate
over I2C to a LCD display.

However, we could try just to use the I2C Arduino Compatible Module."""

channel = 1

# Hold all the address pins high
if smbus is not None:
    try:
        bus = smbus.SMBus(1)
    except Exception as e:
        print(f"SMBus initialization error: {e}")
        bus = None
        smbus = None
else:
    bus = None
    
module_address = 0x27

#Sensor GPIO
TRIG_PIN = 17
ECHO_PIN = 27

#LCD Constants
LCD_CHR = 1
LCD_CMD = 0


LCD_LINE_1 = 0x80
LCD_LINE_2 = 0xC0
LCD_LINE_3 = 0x94
LCD_LINE_4 = 0xD4

#Timing Constnts
E_PULSE = 0.0005
E_DELAY = 0.0005

#Backlight
LCD_BACKLIGHT = 0x08

# Mock to try it on Desktop
try:
    import RPi.GPIO as GPIO
except ImportError:

    class MockGPIO:
        BCM = "BCM"
        BOARD = "BOARD"
        OUT = "OUT"
        IN = "IN"

        def setmode(self, mode):
            print(f"[MOCK] setmode({mode})")

        def setwarnings(self, flag):
            print(f"[MOCK] setwarnings({flag})")

        def setup(self, pin, mode):
            print(f"[MOCK] setup(pin={pin}, mode={mode})")

        def output(self, pin, value):
            print(f"[MOCK] output(pin={pin}, value={value})")

        def input(self, pin):
            # Mock sensor readings - return random values for testing
            import random
            return random.choice([0, 1])

        def cleanup(self):
            print("[MOCK] cleanup()")

        class PWM:
            def __init__(self, pin, freq):
                self.pin = pin
                self.freq = freq
                print(f"[MOCK] PWM(pin={pin}, freq={freq})")

            def start(self, duty):
                print(f"[MOCK] start(duty={duty})")

            def ChangeDutyCycle(self, duty):
                print(f"[MOCK] ChangeDutyCycle(duty={duty})")

            def stop(self):
                print(f"[MOCK] stop()")

    GPIO = MockGPIO()


class GPIOInit:
   
    def __init__(self, servo_pin=12):
        GPIO.setmode(GPIO.BCM)
        self.servo_pin = servo_pin
        GPIO.setup(self.servo_pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.servo_pin, 50)
        self.pwm.start(0)
        
    def set_angle(self, angle):
        duty = 2.5 + (angle / 180.0) * 10
        self.pwm.ChangeDutyCycle(duty)
        time.sleep(0.3)
        self.pwm.ChangeDutyCycle(0)
        
        


        

# Walk Function

# Angle function

def wave(servo_controller, wave_count=3, wave_speed=0.5):
    """
    Makes the servo perform a waving motion
    
    Args:
        servo_controller: GPIOInit instance for the servo
        wave_count: Number of complete waves (back and forth)
        wave_speed: Speed of waving (delay between movements)
    """
    try:
        for _ in range(wave_count):
            # Wave to the right
            servo_controller.set_angle(45)
            time.sleep(wave_speed)
            
            # Wave to the left
            servo_controller.set_angle(135)
            time.sleep(wave_speed)
            
        # Return to neutral position
        servo_controller.set_angle(90)
        time.sleep(wave_speed)
        
    except Exception as e:
        print(f"Error during waving: {e}")
        # Return to neutral position on error
        try:
            servo_controller.set_angle(90)
        except:
            pass


def attack_mode(servo_controller):
    """
    Attack mode - aggressive behaviors when someone gets too close for too long
    
    Args:
        servo_controller: GPIOInit instance for the servo
    """
    try:
        # Stop any playing music
        if pygame and pygame.mixer.music.get_busy():
            pygame.mixer.music.stop()
        
        # Display threatening messages
        lcd_display_string("BACK OFF!", 1)
        lcd_display_string("TOO CLOSE!", 2)
        time.sleep(1)
        
        # Aggressive servo movements
        for _ in range(5):
            servo_controller.set_angle(0)
            time.sleep(0.2)
            servo_controller.set_angle(180)
            time.sleep(0.2)
            
        # More threatening messages
        lcd_display_string("DANGER! DANGER!", 1)
        lcd_display_string("PERSONAL SPACE!", 2)
        time.sleep(2)
        
        # Rapid movements
        for _ in range(8):
            servo_controller.set_angle(random.randint(0, 180))
            time.sleep(0.1)
            
        # Final warning
        lcd_display_string("LEAVE ME ALONE!", 1)
        lcd_display_string("GO AWAY NOW!", 2)
        time.sleep(2)
        
        # Return to neutral position
        servo_controller.set_angle(90)
        
    except Exception as e:
        print(f"Error during attack mode: {e}")
        try:
            servo_controller.set_angle(90)
        except:
            pass

def lcd_toggle_enable(data):
    if smbus is None:
        return
    try:
        bus.write_byte(module_address, data | 0x04)
        time.sleep(E_PULSE)
        bus.write_byte(module_address, data &~0x04)
        time.sleep(E_DELAY)
    except Exception as e:
        print(f"LCD error: {e}")

def lcd_send_byte(bits, mode):
    if smbus is None:
        return
    try:
        #Send high nibble
        high = mode | (bits & 0xF0) | LCD_BACKLIGHT
        bus.write_byte(module_address, high)
        lcd_toggle_enable(high)
        
        low = mode | ((bits << 4) & 0xF8) | LCD_BACKLIGHT
        bus.write_byte(module_address, low)
        lcd_toggle_enable(low)
    except Exception as e:
        print(f"LCD error: {e}")

def lcd_init():
    """Initialize the LCD display"""
    if smbus is None:
        print("[MOCK] LCD not available - using console output")
        return
    try:
        lcd_send_byte(0x33, LCD_CMD)
        lcd_send_byte(0x32, LCD_CMD)
        lcd_send_byte(0x06, LCD_CMD)
        lcd_send_byte(0x0C, LCD_CMD)
        lcd_send_byte(0x28, LCD_CMD)
        lcd_send_byte(0x01, LCD_CMD)
        time.sleep(E_DELAY)
    except Exception as e:
        print(f"LCD initialization error: {e}")

def lcd_display_string(message, line):
    """Send string to display"""
    if smbus is None:
        print(f"[LCD Line {line}] {message}")
        return
    
    try:
        message = message.ljust(16," ")
        if line == 1:
            lcd_send_byte(LCD_LINE_1, LCD_CMD)
        elif line == 2:
            lcd_send_byte(LCD_LINE_2, LCD_CMD)
        elif line ==3:
            lcd_send_byte(LCD_LINE_3, LCD_CMD)
        elif line ==4:
            lcd_send_byte(LCD_LINE_4, LCD_CMD)
            
        for char in message:
            lcd_send_byte(ord(char), LCD_CHR)
    except Exception as e:
        print(f"[LCD Line {line}] {message} (Error: {e})")



#Audio Scripting

def play_mp3(file_path):
    #Use the path to the file I think from the Absolute path
    if pygame is None:
        print(f"[MOCK AUDIO] Would play: {file_path}")
        return
    
    try:
        if not pygame.mixer.get_init():
            pygame.mixer.init()
        
        pygame.mixer.music.load(file_path)
        pygame.mixer.music.set_volume(1)
        pygame.mixer.music.play()
        print(f"Playing: {file_path}")
    except Exception as e:
        print(f"Error Playing Music: {e}")
        
        
#Proximity Settings and determination
def sense_setup():

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(TRIG_PIN, GPIO.OUT)
    GPIO.setup(ECHO_PIN, GPIO.IN)

def get_distance():
    timeout = 0.03
    
    
    GPIO.output(TRIG_PIN, True)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN, False)
    
    start_time = time.time()
    timeout_start = time.time()
    stop_time = time.time()
    while (GPIO.input(ECHO_PIN) == 0):
         start_time = time.time()
         if start_time - timeout_start > timeout:
             return -1
    
    stop_time = time.time()
    timeout_start = stop_time
    
    while (GPIO.input(ECHO_PIN) == 1):
        stop_time = time.time()
        if ((stop_time - timeout_start) > timeout):
            return -1
    
    elapsed = stop_time - start_time 
    distance = (elapsed*34300)/2
    
    return distance
    
def am_u_too_close():
    
    i = 0
    distances = []
    avg_distance = 0
    for i in range(4):
        ret = get_distance()
        if(ret > -1):
           distances.append(ret)
           
    if (len(distances) == 0):
        return False
    else:
        i = 0
        for i in range(len(distances)):
            print(f"distance: {distances[i]}")
            avg_distance = avg_distance + distances[i]
        
        avg_distance = avg_distance/(len(distances))
        
        if(avg_distance < 30):
            return True
        
        else:
            return False
            


def initialize_face_detection():
    """Initialize the face detection system"""
    global face_recognition_system
    
    # Skip the complex face recognition system - we only need basic detection
    print("Skipping complex face recognition system - using direct OpenCV detection")
    return True

def detect_face_quick():
    """
    Quick face detection - checks if any face is present without recognition
    Returns True if a face is detected, False otherwise
    Uses optimizations for real-time performance
    """
    try:
        # Initialize video capture
        video_capture = cv2.VideoCapture(0)
        
        if not video_capture.isOpened():
            print("Camera not available")
            return False
        
        # Set camera properties for faster capture
        video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        video_capture.set(cv2.CAP_PROP_FPS, 15)
        
        # Capture a single frame
        ret, frame = video_capture.read()
        video_capture.release()
        
        if not ret:
            print("Failed to capture frame")
            return False
        
        # Resize frame for faster processing
        small_frame = cv2.resize(frame, (160, 120))
        
        # Convert to grayscale for face detection
        gray = cv2.cvtColor(small_frame, cv2.COLOR_BGR2GRAY)
        
        # Create face cascade classifier directly
        face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
        
        # Detect faces using the cascade classifier with optimized parameters
        faces = face_cascade.detectMultiScale(
            gray, 
            scaleFactor=1.2,  # Slightly larger scale factor for speed
            minNeighbors=3,   # Reduced for faster detection
            minSize=(20, 20), # Smaller minimum size for small frame
            flags=cv2.CASCADE_SCALE_IMAGE
        )
        
        # Return True if any faces detected
        face_detected = len(faces) > 0
        if face_detected:
            print(f"Face detected! Found {len(faces)} face(s)")
        
        return face_detected
        
    except Exception as e:
        print(f"Error in face detection: {e}")
        return False


#FSM States:

def idle():
    #Idle should occur after initialisation 
    if pygame and pygame.mixer.music.get_busy():
        time.sleep(1)
    else:
        play_mp3("resources/audio/spamton dance.mp3")
    lcd_display_string("I am so Alone", 1)
    lcd_display_string("Bottom Text", 2)
    
    # Break up the 10 second wait with condition checks
    for _ in range(10):
        time.sleep(1)
        # Check if conditions changed during wait
        proximate = am_u_too_close()
        face = detect_face_quick()
        if proximate or face:
            if not proximate and face:
                return 1  # FDF
            elif proximate and not face:
                return 2  # NFDC
            elif proximate and face:
                return 3  # FDC
    
    lcd_display_string("", 1)
    lcd_display_string("", 2)
    time.sleep(1)
    
    rand = 1
    proximate = am_u_too_close()
    face = detect_face_quick()  # Use actual face detection
    
    #Put facial detection 
    while((proximate == 0) & (face == 0)): #&&face_in_frame()
        if(rand == 1):
         lcd_display_string("Please talk to me", 1)
         lcd_display_string("I'm so lonely...", 2)
      
        elif(rand == 2):
         lcd_display_string("HooWee Mama am I lonely", 1)
         lcd_display_string("Tirana is the capital of -", 2)
      
        elif(rand == 3):
         lcd_display_string("Nobody loves me", 1)
         lcd_display_string("Forever alone...", 2)
         
        elif(rand == 4):
         play_mp3("resources/audio/HEY EVERY !.mp3")
         lcd_display_string("HEY EVERYBODY!", 1)
         lcd_display_string("PLEASE NOTICE ME!", 2)
         
        # Cycle through random behaviors
        rand = (rand % 4) + 1
        
        # Check conditions more frequently - every 1 second instead of 5
        for _ in range(5):  # 5 checks over 5 seconds
            time.sleep(1)
            proximate = am_u_too_close()
            face = detect_face_quick()  # Update face detection
            if proximate or face:  # Exit early if conditions change
                break
     
    if((proximate == 0) & (face == 0)):
        return 0
    elif((proximate == 0) & (face == 1)):
        return 1
    elif((proximate == 1) & (face == 0)):
        return 2
    elif((proximate == 1) & (face == 1)):
        return 3
    
    
def check_current_condition():
    """
    Check current sensor conditions and return appropriate state code
    Returns:
    0 = Idle (no face, not close)
    1 = FDF (face detected, not close) 
    2 = NFDC (no face, close)
    3 = FDC (face detected, close)
    """
    proximate = am_u_too_close()
    face = detect_face_quick()  # Use actual face detection
    
    if not proximate and not face:
        return 0  # Idle
    elif not proximate and face:
        return 1  # FDF
    elif proximate and not face:
        return 2  # NFDC
    elif proximate and face:
        return 3  # FDC


def FDF():
    #Face Detected and Far
    #Can transition to FDC if person gets close, or back to idle if face lost
    
    servo2 = GPIOInit(servo_pin=13)
    
    if pygame and pygame.mixer.music.get_busy():
        time.sleep(1)
    else:
        play_mp3("resources/audio/spamton dance.mp3")
          
    lcd_display_string("Hey Hey Hey!", 1)
    lcd_display_string("Hello friend!", 2)
    
    # Friendly wave when detecting face at safe distance
    wave(servo2, wave_count=3, wave_speed=0.4)
    
    # Break up the 3 second wait with condition checks
    for _ in range(3):
        time.sleep(1)
        current_state = check_current_condition()
        if current_state != 1:  # Not FDF anymore
            return current_state
    
    # Stay in FDF while face detected and far
    while True:
        current_state = check_current_condition()
        
        if current_state == 3:  # Face detected and close -> transition to FDC
            return 3
        elif current_state == 0:  # No face, not close -> back to idle
            return 0
        elif current_state == 2:  # No face but close -> transition to NFDC
            return 2
        elif current_state == 1:  # Still FDF - continue friendly behavior
            lcd_display_string("I have a good feeling about you!", 1)
            lcd_display_string("I'm so happy to see you!", 2)
            
            # Break up the 3 second wait with frequent checks
            for _ in range(3):
                time.sleep(1)
                current_state = check_current_condition()
                if current_state != 1:  # Exit early if state changes
                    return current_state
            
            # Occasional friendly wave
            if random.randint(1, 3) == 1:
                wave(servo2, wave_count=1, wave_speed=0.5)
            
            lcd_display_string("I love you!", 1)
            lcd_display_string("<3", 2)
            
            # Break up another 3 second wait with checks
            for _ in range(3):
                time.sleep(1)
                current_state = check_current_condition()
                if current_state != 1:  # Exit early if state changes
                    return current_state
        else:
            # Unexpected state, return to idle
            return 0

def FDC():
    #Face is close
    #Can transition to FDF if person moves away, NFDC if face lost, or stay in FDC
    #Will trigger attack mode after 8 seconds
    
    servo2 = GPIOInit(servo_pin=13)
    
    if pygame and pygame.mixer.music.get_busy():
        time.sleep(1)
    else:
        play_mp3("resources/audio/HEY EVERY !.mp3")
    
    lcd_display_string("Hey Hey Hey!", 1)
    lcd_display_string("Nice to see you!", 2)
    
    # Friendly wave when first detecting close face
    wave(servo2, wave_count=2, wave_speed=0.3)
    
    # Timer for attack mode
    close_face_start_time = time.time()
    attack_mode_triggered = False
    
    while True:
        current_state = check_current_condition()
        current_time = time.time()
        time_elapsed = current_time - close_face_start_time
        
        # Check for state transitions first
        if current_state == 1:  # Face detected but far -> transition to FDF
            return 1
        elif current_state == 0:  # No face, not close -> back to idle
            return 0
        elif current_state == 2:  # No face but still close -> transition to NFDC
            return 2
        elif current_state == 3:  # Still FDC - continue behavior
            
            # Check if 8 seconds have passed for attack mode
            if time_elapsed >= 8.0 and not attack_mode_triggered:
                attack_mode(servo2)
                attack_mode_triggered = True
                
                # Reset timer for potential future attacks
                close_face_start_time = current_time
                time.sleep(3)  # Cool down period
                
            elif not attack_mode_triggered:
                # Normal friendly behavior before attack mode
                if time_elapsed < 3:
                    lcd_display_string("Hello there!", 1)
                    lcd_display_string("I'm happy to see you!", 2)
                elif time_elapsed < 6:
                    lcd_display_string("You're quite close...", 1)
                    lcd_display_string("Everything okay?", 2)
                else:
                    lcd_display_string("Getting nervous...", 1)
                    lcd_display_string("Personal space?", 2)
                
                # Check conditions every 0.5 seconds for quick response
                time.sleep(0.5)
                
            else:
                # After attack mode, be more hostile
                lcd_display_string("Still here?", 1)
                lcd_display_string("I warned you!", 2)
                
                # Break up the 2 second wait with checks
                for _ in range(4):  # 4 checks over 2 seconds
                    time.sleep(0.5)
                    current_state = check_current_condition()
                    if current_state != 3:  # Exit early if state changes
                        return current_state
        else:
            # Unexpected state, return to idle
            return 0

def NFDC():
    #No face detected but something close - panic mode
    #Can transition to FDC if face appears, or idle if object moves away
    
    servo2 = GPIOInit(servo_pin=13)
    
    # Stop music and display warning
    if pygame and pygame.mixer.music.get_busy():
        pygame.mixer.music.stop()
    
    lcd_display_string("WHAT'S THAT?!", 1)
    lcd_display_string("WHO'S THERE?!", 2)
    
    # Look around frantically
    for _ in range(3):
        servo2.set_angle(45)
        time.sleep(0.3)
        servo2.set_angle(135)
        time.sleep(0.3)
    
    servo2.set_angle(90)  # Return to center
    
    # Timer for escalating panic
    panic_start_time = time.time()
    
    while True:
        current_state = check_current_condition()
        current_time = time.time()
        time_elapsed = current_time - panic_start_time
        
        # Check for state transitions
        if current_state == 3:  # Face detected and close -> transition to FDC
            return 3
        elif current_state == 1:  # Face detected but far -> transition to FDF
            return 1
        elif current_state == 0:  # No object detected -> back to idle
            lcd_display_string("Phew...", 1)
            lcd_display_string("That was scary!", 2)
            # Break up the 3 second wait with checks
            for _ in range(6):  # 6 checks over 3 seconds
                time.sleep(0.5)
                current_state = check_current_condition()
                if current_state != 0:  # Exit early if state changes
                    return current_state
            return 0
        elif current_state == 2:  # Still NFDC - continue panic behavior
            
            if time_elapsed < 3:
                lcd_display_string("I can't see you!", 1)
                lcd_display_string("Show yourself!", 2)
                # Check conditions every 0.5 seconds
                for _ in range(2):  # 2 checks over 1 second
                    time.sleep(0.5)
                    current_state = check_current_condition()
                    if current_state != 2:  # Exit early if state changes
                        return current_state
            elif time_elapsed < 6:
                lcd_display_string("BACK AWAY!", 1)
                lcd_display_string("DANGER!", 2)
                # Rapid movements
                servo2.set_angle(random.randint(30, 150))
                time.sleep(0.5)
                # Check conditions after movement
                current_state = check_current_condition()
                if current_state != 2:
                    return current_state
            else:
                # Full panic mode
                attack_mode(servo2)
                panic_start_time = current_time  # Reset timer
        else:
            # Unexpected state, return to idle
            return 0

 
    
    
    
    

def main():
    print("Initializing robot systems...")
    
    # Initialize face detection system (optional)
    print("Initializing face detection...")
    try:
        if initialize_face_detection():
            print("✓ Face detection system ready (using direct OpenCV)")
        else:
            print("⚠ Face detection system not available")
    except Exception as e:
        print(f"⚠ Face detection initialization failed: {e}")
    
    # Test basic OpenCV and camera
    print("Testing camera and OpenCV...")
    try:
        import cv2
        print(f"✓ OpenCV version: {cv2.__version__}")
        
        # Test camera quickly
        cap = cv2.VideoCapture(0)
        if cap.isOpened():
            ret, frame = cap.read()
            cap.release()
            if ret:
                print("✓ Camera working")
            else:
                print("⚠ Camera not capturing frames")
        else:
            print("⚠ Camera not available")
            
        # Test face cascade
        face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
        if not face_cascade.empty():
            print("✓ Face detection cascade loaded")
        else:
            print("⚠ Face detection cascade not loaded")
            
    except Exception as e:
        print(f"⚠ Camera/OpenCV test failed: {e}")
    
    # Initialize other systems
    try:
        lcd_init()
        print("✓ LCD initialized")
    except Exception as e:
        print(f"⚠ LCD initialization failed: {e}")
    
    try:
        sense_setup()
        print("✓ Proximity sensor initialized")
    except Exception as e:
        print(f"⚠ Proximity sensor initialization failed: {e}")
    
    # Test face detection
    print("Testing face detection...")
    try:
        face_test = detect_face_quick()
        print(f"✓ Face detection test: {'PASSED' if face_test else 'No faces detected'}")
    except Exception as e:
        print(f"⚠ Face detection test failed: {e}")
    
    # Initialize audio thread
    try:
        music_thread = threading.Thread(target=play_mp3, args=("resources/audio/HEY EVERY !.mp3",), daemon = True)
        music_thread.start()
        print("✓ Audio system initialized")
    except Exception as e:
        print(f"⚠ Audio initialization failed: {e}")
    
    # Start in idle state
    current_state = 0
    
    print("Robot State Machine Starting...")
    print("Press Ctrl+C to stop the robot")
    
    try:
        while True:
            if current_state == 0:
                print("State: Idle")
                current_state = idle()
            elif current_state == 1:
                print("State: Face Detected Far (FDF)")
                current_state = FDF()
            elif current_state == 2:
                print("State: No Face Detected Close (NFDC)")
                current_state = NFDC()
            elif current_state == 3:
                print("State: Face Detected Close (FDC)")
                current_state = FDC()
            else:
                # Invalid state, return to idle
                print(f"Invalid state {current_state}, returning to idle")
                current_state = 0
            
            time.sleep(0.05)  # Reduced delay for faster state transitions
    except KeyboardInterrupt:
        print("\nRobot stopped by user")
    except Exception as e:
        print(f"Robot error: {e}")
    finally:
        try:
            GPIO.cleanup()
            print("GPIO cleanup completed")
        except:
            pass
        

        


 
                                                                                                                                 
if __name__ == "__main__":
    main()
