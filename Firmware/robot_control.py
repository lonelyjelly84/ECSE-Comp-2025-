import time, math

import time, math

# Mock to try it on Desktop
try:
    import lgpio
    # For servo control, we'll use the hardware PWM channels available on Pi
    # Pi has 2 hardware PWM channels on GPIO 12 and 13 (which map to BOARD pins 32 and 33)
    # But since you're using BOARD pins 12 and 13, let's use GPIO chip with software PWM alternative
    gpio_chip = lgpio.gpiochip_open(0)
    GPIO_AVAILABLE = True
except ImportError:
    class MockLGPIO:
        def gpiochip_open(self, chip):
            print(f"[MOCK] gpiochip_open({chip})")
            return 0
            
        def gpiochip_close(self, handle):
            print(f"[MOCK] gpiochip_close({handle})")
            
        def gpio_claim_output(self, handle, gpio, level=0):
            print(f"[MOCK] gpio_claim_output(handle={handle}, gpio={gpio}, level={level})")
            
        def gpio_write(self, handle, gpio, level):
            print(f"[MOCK] gpio_write(handle={handle}, gpio={gpio}, level={level})")
            
        def gpio_free(self, handle, gpio):
            print(f"[MOCK] gpio_free(handle={handle}, gpio={gpio})")

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

    lgpio = MockLGPIO()
    gpio_chip = lgpio.gpiochip_open(0)
    GPIO_AVAILABLE = False

# Convert BOARD pin numbers to GPIO numbers
# BOARD 12 = GPIO 18, BOARD 13 = GPIO 27
BOARD_TO_GPIO = {12: 18, 13: 27}
servo_board_pins = [12, 13]  # BOARD pin numbers for arm servos
servo_gpio_pins = [BOARD_TO_GPIO[pin] for pin in servo_board_pins]  # Convert to GPIO numbers
FREQ = 50  # 50Hz for SG90

# Software PWM implementation for lgpio (Pi 4 optimized)
class SoftwarePWM:
    def __init__(self, gpio_chip, pin, freq):
        self.gpio_chip = gpio_chip
        self.pin = pin
        self.freq = freq
        self.period = 1.0 / freq  # 20ms for 50Hz
        self.duty_cycle = 0
        self.running = False
        self.current_angle = 90  # Track current servo position
        
        if GPIO_AVAILABLE:
            lgpio.gpio_claim_output(gpio_chip, pin, 0)
        print(f"[PWM] Setup pin {pin} at {freq}Hz for Pi 4")
    
    def start(self, duty):
        self.duty_cycle = duty
        self.running = True
        print(f"[PWM] Started pin {self.pin} with duty {duty}%")
    
    def ChangeDutyCycle(self, duty):
        """Update servo position with optimized timing for Pi 4"""
        self.duty_cycle = duty
        
        if GPIO_AVAILABLE and self.running:
            # Convert duty cycle to servo angle (2.5% = 0°, 12.5% = 180°)
            # For SG90: 1ms = 0°, 1.5ms = 90°, 2ms = 180°
            pulse_width_ms = 1.0 + (duty - 2.5) * (1.0 / 10.0)  # 1-2ms range
            pulse_width_s = pulse_width_ms / 1000.0
            
            # Send multiple pulses for stable positioning (Pi 4 optimization)
            for _ in range(3):  # Send 3 pulses for reliable positioning
                lgpio.gpio_write(self.gpio_chip, self.pin, 1)
                time.sleep(pulse_width_s)
                lgpio.gpio_write(self.gpio_chip, self.pin, 0)
                time.sleep(self.period - pulse_width_s)  # Complete the 20ms cycle
            
            # Calculate and store current angle
            self.current_angle = ((duty - 2.5) / 10.0) * 180.0
            print(f"[PWM] Pin {self.pin} moved to {self.current_angle:.1f}° (duty: {duty:.1f}%)")
        else:
            print(f"[PWM] Mock: Pin {self.pin} duty to {duty}%")
    
    def stop(self):
        self.running = False
        if GPIO_AVAILABLE:
            lgpio.gpio_write(self.gpio_chip, self.pin, 0)
        print(f"[PWM] Stopped pin {self.pin}")
    
    def get_angle(self):
        """Get current servo angle"""
        return self.current_angle

# Setup PWM for servos
pwms = []
for i, gpio_pin in enumerate(servo_gpio_pins):
    if GPIO_AVAILABLE:
        pwm = SoftwarePWM(gpio_chip, gpio_pin, FREQ)
    else:
        # Use mock PWM for development
        pwm = lgpio.PWM(servo_board_pins[i], FREQ)
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
        if GPIO_AVAILABLE:
            # Free GPIO pins
            for gpio_pin in servo_gpio_pins:
                lgpio.gpio_free(gpio_chip, gpio_pin)
            lgpio.gpiochip_close(gpio_chip)


if __name__ == "__main__":
    main()
