"""
ECSE Robot Sensor Navigation System (Fallback Option)
Ultrasonic Sensor → Distance Detection → Movement Commands
"""

import time
import sys
import os
from typing import Optional

# Add firmware path to access robot controller
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'Firmware'))
from robot_control import ECSERobot

# Mock GPIO for development
try:
    import RPi.GPIO as GPIO
except ImportError:
    class MockGPIO:
        BCM = "BCM"
        BOARD = "BOARD"
        OUT = "OUT"
        IN = "IN"
        HIGH = 1
        LOW = 0

        def setmode(self, mode):
            print(f"[MOCK] GPIO.setmode({mode})")

        def setup(self, pin, mode):
            print(f"[MOCK] GPIO.setup(pin={pin}, mode={mode})")

        def output(self, pin, state):
            print(f"[MOCK] GPIO.output(pin={pin}, state={state})")

        def input(self, pin):
            print(f"[MOCK] GPIO.input(pin={pin}) -> 0")
            return 0

        def cleanup(self):
            print("[MOCK] GPIO.cleanup()")

    GPIO = MockGPIO()


class UltrasonicSensor:
    """Handle HC-SR04 ultrasonic sensor"""
    
    def __init__(self, trigger_pin=18, echo_pin=24):
        self.trigger_pin = trigger_pin
        self.echo_pin = echo_pin
        self.max_distance = 400  # cm
        
    def initialize(self):
        """Initialize GPIO pins for sensor"""
        try:
            GPIO.setmode(GPIO.BOARD)
            GPIO.setup(self.trigger_pin, GPIO.OUT)
            GPIO.setup(self.echo_pin, GPIO.IN)
            print(f"Ultrasonic sensor initialized on pins {self.trigger_pin}/{self.echo_pin}")
            return True
        except Exception as e:
            print(f"Sensor init error: {e}")
            return False
    
    def get_distance(self) -> Optional[float]:
        """Get distance measurement in centimeters"""
        try:
            # Send trigger pulse
            GPIO.output(self.trigger_pin, GPIO.HIGH)
            time.sleep(0.00001)  # 10 microseconds
            GPIO.output(self.trigger_pin, GPIO.LOW)
            
            # Wait for echo start
            pulse_start = time.time()
            timeout = pulse_start + 0.1  # 100ms timeout
            
            while GPIO.input(self.echo_pin) == 0:
                if time.time() > timeout:
                    return None
                pulse_start = time.time()
            
            # Wait for echo end
            pulse_end = time.time()
            timeout = pulse_end + 0.1
            
            while GPIO.input(self.echo_pin) == 1:
                if time.time() > timeout:
                    return None
                pulse_end = time.time()
            
            # Calculate distance
            pulse_duration = pulse_end - pulse_start
            distance = pulse_duration * 17150  # Speed of sound calculation
            
            if 2 <= distance <= self.max_distance:
                return distance
            return None
            
        except Exception as e:
            print(f"Distance measurement error: {e}")
            return None
    
    def cleanup(self):
        """Clean up GPIO"""
        GPIO.cleanup()


class SensorNavigationController:
    """Navigation logic using ultrasonic sensor data"""
    
    def __init__(self, safe_distance=30.0, turn_distance=50.0):
        self.safe_distance = safe_distance  # cm
        self.turn_distance = turn_distance  # cm
    
    def get_navigation_command(self, distance: Optional[float]) -> str:
        """Determine movement based on distance reading"""
        if distance is None:
            return "stop"  # No valid reading
        
        if distance < self.safe_distance:
            return "turn_left"  # Obstacle too close
        elif distance < self.turn_distance:
            return "turn_right"  # Obstacle ahead, turn right
        else:
            return "forward"  # Path clear


class SensorNavigator:
    """Main sensor navigation system"""
    
    def __init__(self, trigger_pin=18, echo_pin=24):
        self.sensor = UltrasonicSensor(trigger_pin, echo_pin)
        self.controller = SensorNavigationController()
        self.robot = ECSERobot()
        self.running = False
        
    def initialize(self):
        """Initialize all sensor systems"""
        print("Initializing ECSE Robot Sensor Navigator...")
        
        if not self.sensor.initialize():
            print("Sensor initialization failed")
            return False
            
        print("Sensor system initialized successfully")
        return True
    
    def run(self, max_iterations=100):
        """Main sensor navigation loop"""
        self.running = True
        iteration = 0
        
        print(f"Starting sensor navigation (max {max_iterations} iterations)")
        
        try:
            while self.running and iteration < max_iterations:
                # Get distance reading
                distance = self.sensor.get_distance()
                
                if distance is not None:
                    print(f"Distance: {distance:.1f}cm")
                
                # Get navigation command
                command = self.controller.get_navigation_command(distance)
                print(f"Command: {command}")
                
                # Execute movement
                if command == "forward":
                    self.robot.forward()
                elif command == "turn_left":
                    self.robot.turn_left()
                elif command == "turn_right":
                    self.robot.turn_right()
                else:  # stop
                    self.robot.stop()
                
                time.sleep(0.2)  # Brief pause between readings
                iteration += 1
                
        except KeyboardInterrupt:
            print("Navigation interrupted by user")
        except Exception as e:
            print(f"Navigation error: {e}")
        finally:
            self.stop()
    
    def stop(self):
        """Stop and cleanup"""
        self.running = False
        self.robot.stop()
        self.sensor.cleanup()
        print("Sensor system stopped")


def main():
    """Development/testing entry point - use main.py for normal operation"""
    print("Sensor Navigation - Development Mode")
    print("For normal operation, run: python main.py")
    print("=" * 50)
    
    navigator = SensorNavigator(trigger_pin=18, echo_pin=24)
    
    if navigator.initialize():
        navigator.run(max_iterations=30)
    else:
        print("Failed to initialize sensor system")


if __name__ == "__main__":
    main()