import time
import math

# Use gpiozero for servo control (pre-installed on RPi)
try:
    from gpiozero import Servo
    from gpiozero.pins.pigpio import PiGPIOFactory
    
    # Try to use pigpio for better servo control, fallback to default
    try:
        pin_factory = PiGPIOFactory()
        GPIO_AVAILABLE = True
        print("[INFO] Using pigpio pin factory for precise servo control")
    except:
        pin_factory = None
        GPIO_AVAILABLE = True
        print("[INFO] Using default pin factory for servo control")
        
except ImportError:
    # Mock for desktop testing
    class MockServo:
        def __init__(self, pin):
            self.pin = pin
            self._value = 0
            print(f"[MOCK] Servo created on pin {pin}")
            
        @property
        def value(self):
            return self._value
            
        @value.setter
        def value(self, val):
            self._value = val
            print(f"[MOCK] Servo pin {self.pin} set to {val}")
            
        def close(self):
            print(f"[MOCK] Servo pin {self.pin} closed")
    
    Servo = MockServo
    pin_factory = None
    GPIO_AVAILABLE = False

# Convert BOARD pin numbers to GPIO numbers (for gpiozero)
# BOARD pins 12, 13 = GPIO pins 18, 19
servo_gpio_pins = [18, 19]  # GPIO pins for servo control

# Create servo objects
servo1 = None
servo2 = None

def setup_servos():
    """Initialize servo objects"""
    global servo1, servo2
    
    if GPIO_AVAILABLE:
        if pin_factory:
            servo1 = Servo(servo_gpio_pins[0], pin_factory=pin_factory)
            servo2 = Servo(servo_gpio_pins[1], pin_factory=pin_factory)
        else:
            servo1 = Servo(servo_gpio_pins[0])
            servo2 = Servo(servo_gpio_pins[1])
        print(f"[INFO] Servos initialized on GPIO pins {servo_gpio_pins}")
    else:
        servo1 = Servo(servo_gpio_pins[0])
        servo2 = Servo(servo_gpio_pins[1])
        print("[MOCK] Servo objects created")

# Initialize servos
setup_servos()

def move_servo_smooth(servo, start_pos, end_pos, duration=0.5, steps=20):
    """
    Move a servo smoothly from start_pos to end_pos.
    gpiozero servo values range from -1 (0°) to 1 (180°), 0 is 90°
    """
    if not servo:
        print("[ERROR] Servo not initialized")
        return
        
    for i in range(steps + 1):
        t = i / steps
        # Cosine easing for smoother movement
        eased = 0.5 * (1 - math.cos(math.pi * t))
        pos = start_pos + (end_pos - start_pos) * eased
        servo.value = pos
        time.sleep(duration / steps)

def angle_to_servo_value(angle):
    """Convert angle (0-180°) to gpiozero servo value (-1 to 1)"""
    return (angle - 90) / 90.0

# Arm Robot Class
class ArmRobot:
    def __init__(self):
        self.servo1 = servo1
        self.servo2 = servo2
        self.center()

    def center(self):
        """Move both servos to middle (90 degrees = 0 value)"""
        if self.servo1 and self.servo2:
            self.servo1.value = 0  # 90 degrees
            self.servo2.value = 0  # 90 degrees
            time.sleep(0.3)
            print("[INFO] Arms centered")

    def raise_arm(self, duration=1.0):
        """Raise the arm when face is detected"""
        print("Raising arm - Face detected!")
        if self.servo1 and self.servo2:
            # Convert angles to servo values: 45° = -0.5, 135° = 0.5
            move_servo_smooth(self.servo1, 0, angle_to_servo_value(45), duration)
            move_servo_smooth(self.servo2, 0, angle_to_servo_value(135), duration)
            time.sleep(0.2)

    def lower_arm(self, duration=1.0):
        """Lower the arm back to neutral position"""
        print("Lowering arm")
        if self.servo1 and self.servo2:
            move_servo_smooth(self.servo1, self.servo1.value, 0, duration)
            move_servo_smooth(self.servo2, self.servo2.value, 0, duration)
            time.sleep(0.2)

    def wave(self, repetitions=3):
        """Make a waving motion"""
        print("Waving!")
        if self.servo1:
            for _ in range(repetitions):
                move_servo_smooth(self.servo1, 0, angle_to_servo_value(60), 0.3)
                move_servo_smooth(self.servo1, angle_to_servo_value(60), angle_to_servo_value(120), 0.3)
            self.center()

    def sad_droop(self, duration=1.0):
        """Droop arms sadly when alone"""
        print("Drooping arms sadly")
        if self.servo1 and self.servo2:
            move_servo_smooth(self.servo1, 0, angle_to_servo_value(120), duration)
            move_servo_smooth(self.servo2, 0, angle_to_servo_value(60), duration)
            time.sleep(0.2)

    def stop(self):
        """Return to neutral position and close servos"""
        self.center()
        if GPIO_AVAILABLE and self.servo1 and self.servo2:
            self.servo1.close()
            self.servo2.close()
            print("[INFO] Servos closed")


# Test function for arm movements
def main():
    robot = ArmRobot()

    try:
        print("Testing arm movements...")
        robot.raise_arm()
        time.sleep(2)
        robot.wave()
        time.sleep(1)
        robot.sad_droop()
        time.sleep(1)
        robot.lower_arm()
        robot.stop()
    except KeyboardInterrupt:
        print("\nStopping robot...")
        robot.stop()


if __name__ == "__main__":
    main()