import time, math, smbus, pygame, threading, random

import numpy as np

      
"""I2C LCD Display Module, could communicat with an Arduino which can then communicate
over I2C to a LCD display.

However, we could try just to use the I2C Arduino Compatible Module."""

channel = 1

# Hold all the address pins high
bus= smbus.SMBus(1)
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


class GPIOInit:
   
    def __init__(self, servo_pin=12):
        GPIO.setmode(GPIO.BCM)
        self.servo_pin = servo_pin
        GPIO.setup(self.servo_pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.servo_pin, 50)
        self.pwm.start(0)
    
    def __init__(self, servo_pin=13):
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



def lcd_toggle_enable(data):
    bus.write_byte(module_address, data | 0x04)
    time.sleep(E_PULSE)
    bus.write_byte(module_address, data &~0x04)
    time.sleep(E_DELAY)

def lcd_send_byte(bits, mode):
    #Send high nibble
    high = mode | (bits & 0xF0) | LCD_BACKLIGHT
    bus.write_byte(module_address, high)
    lcd_toggle_enable(high)
    
    low = mode | ((bits << 4) & 0xF8) | LCD_BACKLIGHT
    bus.write_byte(module_address, low)
    lcd_toggle_enable(low)

def lcd_init():
    """Initialize the LCD display"""
    lcd_send_byte(0x33, LCD_CMD)
    lcd_send_byte(0x32, LCD_CMD)
    lcd_send_byte(0x06, LCD_CMD)
    lcd_send_byte(0x0C, LCD_CMD)
    lcd_send_byte(0x28, LCD_CMD)
    lcd_send_byte(0x01, LCD_CMD)
    time.sleep(E_DELAY)

def lcd_display_string(message, line):
    """Send string to display"""
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



#Audio Scripting

def play_mp3(file_path):
    #Use the path to the file I think from the Absolute path
    try:
        pygame.mixer.init()
        
        pygame.mixer.music.load(file_path)
        
        pygame.mixer.music.set_volume(1)
        
        pygame.mixer.music.play()
    except pygame.error as e:
        printf(f"Error Playing Music: {e}")
        
        
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
            



#FSM States:

def idle():
    #Idle should occur after initialisation 
    servo2 = GPIOInit(servo_pin=13)
    if(pygame.mixer.music.get_busy()):
            time.sleep(1)
    else:
          play_mp3("spamton dance.mp3")
    lcd_display_string("I am so Alone", 1)
    lcd_display_string("Bottom Text", 2)
    time.sleep(10)
    
    lcd_display_string("", 1)
    lcd_display_string("", 2)
    time.sleep(1)
    
    servo2.set_angle(180)
    time.sleep(1)
    
    rand = 1
    proximate = am_u_too_close()
    face = False
    #Put facial detection 
    while((proximate == 0) & (face == 0)): #&&face_in_frame()
        if(rand == 1):
         lcd_display_string("Please talk to me", 1)
      
        elif(rand == 2):
         lcd_display_string("HooWee Mama am I lonely", 1)
         lcd_display_string("Tirana is the capital of -", 2)
      
        elif(rand == 3):
         servo2.set_angle(180)
         time.sleep(10)
         
        elif(rand == 4):
         play_mp3("HEY EVERY !.mp3")
         
        proximate = am_u_too_close()
        #face = face_in_frame
     
    if((proximate == 0) & (face == 0)):
        return 0
    elif((proximate == 0) & (face == 1)):
        return 1
    elif((proximate == 1) & (face == 0)):
        return 2
    elif((proximate == 1) & (face == 1)):
        return 3
    
    
def FDF():
    #Triumphant!!
    if(pygame.mixer.music.get_busy()):
            time.sleep(1)
    else:
          play_mp3("spamton dance.mp3")
          
    lcd_display_string("Hey Hey Hey! Play the music friend!", 1)
    lcd_display_string("Whats the happening Friendo!", 2)
    time.sleep(10)
    
    while(am_u_too_close() == 0):
        lcd_display_string("I have a good feeling about you!", 1)
        lcd_display_string("I'm so happy to see you and be your friend", 2)
        time.sleep(10)
        
        lcd_display_string("I love you!", 1)
        lcd_display_string("<3", 2)
        time.sleep(10)
 
    
    
    
    

# We should define a Main() and run this on there
def main():
    lcd_init()
    music_thread = threading.Thread(target=play_mp3, args=("/home/josh/Downloads/HEY EVERY !.mp3",), daemon = True)
    sense_setup()
    
    music_thread.start()
    while(1):
        pingas = idle()
        print(f"{pingas}")
        

        


 
                                                                                                                                 
if __name__ == "__main__":
    main()
