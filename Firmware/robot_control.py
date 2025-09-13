import time, math

# Mock to try it on Desktop
try:
    import RPi.GPIO as GPIO
except ImportError:

    class MockGPIO:
        BCM = "BCM"
        BOARD = "BOARD"
        OUT = "OUT"

        def setmode(self, mode):
            print(f"[MOCK] setmode({mode})")

        def setwarnings(self, flag):
            print(f"[MOCK] setwarnings({flag})")

        def setup(self, pin, mode):
            print(f"[MOCK] setup(pin={pin}, mode={mode})")

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

# Servo Configuration for Arm
servo_pins = [12, 13]  # BOARD pin numbers for arm servos
FREQ = 50  # 50Hz for SG90

# Setup GPIO and PWM
GPIO.setmode(GPIO.BOARD)
pwms = []

for pin in servo_pins:
    GPIO.setup(pin, GPIO.OUT)
    pwm = GPIO.PWM(pin, FREQ)
    pwm.start(0)  # start with 0% duty cycle
    pwms.append(pwm)

# Assign arm servo PWM objects
arm_servo1 = pwms[0]
arm_servo2 = pwms[1]


def move_servo(pwm, start_angle, end_angle, duration=0.5, steps=20):
    """
    Move a servo smoothly from start_angle to end_angle.
    Uses cosine easing for smoother movement.
    """
    for i in range(steps + 1):
        t = i / steps
        eased = 0.5 * (1 - math.cos(math.pi * t))  # cosine ease
        angle = start_angle + (end_angle - start_angle) * eased
        duty = 2.5 + (angle / 180.0) * 10
        pwm.ChangeDutyCycle(duty)
        time.sleep(duration / steps)


# Arm Robot Class
class ArmRobot:
    def __init__(self, servo1, servo2):
        self.servo1 = servo1
        self.servo2 = servo2
        self.center()

    def center(self):
        # Move both servos to middle (90 degrees)
        move_servo(self.servo1, 90, 90, duration=0.3)
        move_servo(self.servo2, 90, 90, duration=0.3)
        time.sleep(0.1)

    def raise_arm(self, duration=1.0):
        # Raise the arm when face is detected
        print("Raising arm - Face detected!")
        move_servo(self.servo1, 90, 45, duration=duration)  # Move servo1 upward
        move_servo(self.servo2, 90, 135, duration=duration)  # Move servo2 upward
        time.sleep(0.5)

    def lower_arm(self, duration=1.0):
        # Lower the arm back to neutral position
        print("Lowering arm")
        move_servo(self.servo1, 45, 90, duration=duration)  # Return servo1 to center
        move_servo(self.servo2, 135, 90, duration=duration)  # Return servo2 to center
        time.sleep(0.5)

    def wave(self, repetitions=3):
        # Make a waving motion
        print("Waving!")
        for _ in range(repetitions):
            move_servo(self.servo1, 90, 60, duration=0.3)
            move_servo(self.servo1, 60, 120, duration=0.3)
        self.center()

    def stop(self):
        # Return to neutral position
        self.center()


# Test function for arm movements
def main():
    robot = ArmRobot(arm_servo1, arm_servo2)

    try:
        # Test arm movements
        robot.raise_arm()
        time.sleep(2)
        robot.wave()
        time.sleep(1)
        robot.lower_arm()
        robot.stop()
    finally:
        arm_servo1.stop()
        arm_servo2.stop()
        GPIO.cleanup()


if __name__ == "__main__":
    main()
