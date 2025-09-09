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

# Servo Configuration
servo_pins = [12, 13]  # BOARD pin numbers for left and right servo
FREQ = 50  # 50Hz for SG90

# Setup GPIO and PWM
GPIO.setmode(GPIO.BOARD)
pwms = []

for pin in servo_pins:
    GPIO.setup(pin, GPIO.OUT)
    pwm = GPIO.PWM(pin, FREQ)
    pwm.start(0)  # start with 0% duty cycle
    pwms.append(pwm)

# Assign left and right PWM objects
left_pwm = pwms[0]
right_pwm = pwms[1]


# Walk Function
def walk(pwm, start_angle, end_angle, duration=0.5, steps=20):
    """
    Walk a servo smoothly from start_angle to end_angle.
    Uses cosine easing for smoother gait.
    """
    for i in range(steps + 1):
        t = i / steps
        eased = 0.5 * (1 - math.cos(math.pi * t))  # cosine ease
        angle = start_angle + (end_angle - start_angle) * eased
        duty = 2.5 + (angle / 180.0) * 10
        pwm.ChangeDutyCycle(duty)
        time.sleep(duration / steps)


# Robot Class
class SG90Robot:
    def __init__(self, left_servo, right_servo):
        self.left = left_servo
        self.right = right_servo
        self.center()

    def center(self):
        # Move both servos to middle (90 degrees)
        walk(self.left, 90, 90, duration=0.3)
        walk(self.right, 90, 90, duration=0.3)
        time.sleep(0.1)

    def forward(self, step_time=0.6):
        # Take a forward step (left then right)
        print("Forward")
        walk(self.left, 90, 120, duration=step_time)
        walk(self.right, 90, 90, duration=step_time)

        walk(self.left, 120, 90, duration=step_time)
        walk(self.right, 90, 60, duration=step_time)

    def backward(self, step_time=0.6):
        # Take a backward step (left then right)
        print("Back")
        walk(self.left, 90, 60, duration=step_time)
        walk(self.right, 90, 90, duration=step_time)

        walk(self.left, 60, 90, duration=step_time)
        walk(self.right, 90, 120, duration=step_time)

    def turn_left(self, step_time=0.5):
        # Pivot left in place
        print("left")
        walk(self.left, 90, 60, duration=step_time)
        walk(self.right, 90, 90, duration=step_time)

        walk(self.left, 60, 90, duration=step_time)
        walk(self.right, 90, 60, duration=step_time)

    def turn_right(self, step_time=0.5):
        # Pivot right in place
        print("right")
        walk(self.left, 90, 120, duration=step_time)
        walk(self.right, 90, 90, duration=step_time)

        walk(self.left, 120, 90, duration=step_time)
        walk(self.right, 90, 120, duration=step_time)

    def stop(self):
        # Return to neutral
        self.center()


# We should define a Main() and run this on there
def main():
    robot = SG90Robot(left_pwm, right_pwm)

    try:
        robot.forward()
        robot.turn_left()
        robot.forward()
        robot.turn_right()
        robot.backward()
        robot.stop()
    finally:
        left_pwm.stop()
        right_pwm.stop()
        GPIO.cleanup()


if __name__ == "__main__":
    main()
