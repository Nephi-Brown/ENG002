from machine import Pin, PWM
from time import sleep
import math

# define pins for motors

# motor 1
dir1 = Pin(0,Pin.OUT) # direction pin for motor 1 
pwm1=PWM(Pin(1)) # power pin for motor 1 
pwm1.freq(1000)# sets the pwm for 1000 Hz(turns it on and off 1000 times a second)

# motor 2
dir2 = Pin(2,Pin.OUT) # direction pin for motor 2
pwm2=PWM(Pin(3)) # power pin for motor 2
pwm2.freq(1000)# sets the pwm for 1000 Hz(turns it on and off 1000 times a second)

# Left encoder pins
LA = Pin(4, Pin.IN, Pin.PULL_UP)  # Channel A
LB = Pin(5, Pin.IN, Pin.PULL_UP)  # Channel B

# Right encoder pins
RA = Pin(6, Pin.IN, Pin.PULL_UP)  # Channel A
RB = Pin(7, Pin.IN, Pin.PULL_UP)  # Channel B

class Encoder:
    def __init__(self, pin_a, pin_b):
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.position = 0

        # Attach interrupt to channel A
        self.pin_a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self._callback)

    def _callback(self, pin):
        if self.pin_b.value() == pin.value():
            self.position += 1
        else:
            self.position -= 1

    def reset(self):
        self.position = 0

    def read(self):
        return self.position

def left_motor_control(direction, speed):
    dir1.value(direction) # sets direction of motor 1 
    pwm1.duty_u16(int(65535*speed)) # sets the speed of motor 1
    
def right_motor_control(direction, speed):
    dir2.value(direction) # sets direction of motor 2
    pwm2.duty_u16(int(65535*speed)) # sets the speed of motor 2
    
def stop_robot():
    pwm1.duty_u16(0) 
    pwm2.duty_u16(0)
    dir1.value(0)
    dir2.value(0)

left_encoder = Encoder(LA, LB)
right_encoder = Encoder(RA, RB)

wheel_diameter = 0.032  # meters
wheel_base = 0.10       # distance between wheels in meters
counts_per_rev = 40     # encoder counts per wheel revolution CHANGE THESE

wheel_circumference = math.pi * wheel_diameter

def counts_for_degrees(angle):
    """
    Calculate the number of encoder counts needed for a pivot turn
    angle: positive = left, negative = right
    """
    turn_circumference = math.pi * wheel_base
    distance_per_wheel = turn_circumference * (abs(angle)/360)  # arc length
    rotations = distance_per_wheel / wheel_circumference
    counts = rotations * counts_per_rev
    return int(counts)

def turn_degrees(angle, speed=0.5):
    target_counts = counts_for_degrees(angle)
    
    left_encoder.reset()
    right_encoder.reset()
    
    if angle > 0:  # left turn
        left_motor_control(0, speed)   # left reverse
        right_motor_control(1, speed)  # right forward
    else:          # right turn
        left_motor_control(1, speed)   # left forward
        right_motor_control(0, speed)  # right reverse

    # Wait until one wheel reaches target counts
    while abs(left_encoder.read()) < target_counts:
        pass

    stop_robot()

while True:
    # Turn 90° left
    turn_degrees(90, speed=0.5)
    sleep(0.5)

    # Turn 90° right
    turn_degrees(-90, speed=0.5)
    sleep(0.5)