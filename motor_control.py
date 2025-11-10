from machine import Pin,PWM
from time import sleep

sensor = Pin(15,Pin.IN)

#Motor 1
MR1_PinB=Pin(18,Pin.OUT) #forward pin
MR1_PinA=Pin(17,Pin.OUT) #backward pin
pwm1=PWM(Pin(19))
pwm1.freq(1000)#sets the pwm for 1000 Hz(turns it on and off 1000 times a second)

#Motor 2 (pins need to be changed)
MR2_PinB=Pin(18,Pin.OUT) #forward pin
MR2_PinA=Pin(17,Pin.OUT) #backward pin
pwm2=PWM(Pin(19))
pwm2.freq(1000)#sets the pwm for 1000 Hz(turns it on and off 1000 times a second)

def move_robot(dir1, dir2, speed): # function takes directions for motors and their speed
    # control for motor 1
    if dir1 == "forward":
        MR1_PinA.value(0)
        MR1_PinB.value(1)
    elif dir1 == "backward":
        MR1_PinA.value(1)
        MR1_PinB.value(0)
    else:
        MR1_PinA.value(0)
        MR1_PinB.value(0)
    pwm1.duty_u16(int(speed*65535))
    
    #control for motor 2  
    if dir2 == "forward":
        MR2_PinA.value(0)
        MR2_PinB.value(1)
    elif dir2 == "backward":
        MR2_PinA.value(1)
        MR2_PinB.value(0)
    else:
        MR2_PinA.value(0)
        MR2_PinB.value(0)
    pwm2.duty_u16(int(speed*65535))  
 

while True:
    if sensor.value() == 0:
        print("object detected")
        move_robot("backward", "backward", 0.5) # both motors backwards at 50% speed
    else:
        print("no object")
        move_robot("forward", "forward", 0.5) # both motors forwards at 50% speed 
    sleep(0.5)
 
    
        