from machine import I2C, SoftI2C, Pin, PWM
import time

from vl53l0x import VL53L0X
from vl6180x import VL6180X

i2c = I2C(0, scl=Pin(17), sda=Pin(16)) #sets up for all the tof, they all share one i2c

front = VL6180X(i2c, address=0x29) #this is setting up the address for all the tof sensors, the front one must have address of 0x29
right = VL6180X(i2c, address=0x30)
left = VL6180X(i2c, address=0x31)

#setting up the speed and direction pins for the motor, from the motor controller
dir1 = Pin(23,Pin.OUT) #left motor
dir2 = Pin(24,Pin.OUT) #right motor
pwm1 = PWM(Pin(15))
pwm2 = PWM(Pin(14))
pwm1.freq(1000)
pwm2.freq(1000)

#encoder pins
encoder_l_a=Pin(19,Pin.IN)
encoder_l_b=Pin(20,Pin.IN)
encoder_r_a=Pin(21,Pin.IN)
encoder_r_b=Pin(22,Pin.IN)

#this is a place to store the pulses of the encoders
encoder_countl=0
encoder_countr=0

#encoder interrput callback functions
def encoder_call_left():#if the value of the encoder is 0 it will add a count to the encoder_count(moving forward) if not it will minus one, this works as the function is only called when there is a change in the pin, which must mean the motor is moving
    global encoder_countl
    if encoder_l_b.value() == 0:
        encoder_countl +=1
    else:
        encoder_countl -=1
    
def encoder_call_right():
    global encoder_countr
    if encoder_r_b.value() == 0:
        encoder_countr +=1
    else:
        encoder_countr -=1
  
#interrputs for the encoders
encoder_l_a.irq(trigger=Pin.IRQ.RISING, handler=encoder_call_left) #whenever a pulse is detetecd from the encoder it runs the encoder call functions

encoder_r_a.irq(trigger=Pin.IRQ.RISING, handler=encoder_call_right)

def turn_left(count_target):
    global encoder_countl, encoder_countr
    encoder_countl=0
    encoder_countr=0
    
    dir1.value(0)
    pwm1.duty_u16(0)
    dir2.value(1)
    pwm2.duty_u16(32768)
    
    while encoder_countr< count_target: #waiting for this condition to become false
        pass
    stop()

def turn_right(count_target):
    global encoder_countl, encoder_countr
    encoder_countl=0
    encoder_countr=0
    
    dir1.value(1)
    pwm1.duty_u16(32768)
    dir2.value(0)
    pwm2.duty_u16(0)
    
    while encoder_countl< count_target: 
        pass
    stop()

def stop():
    dir1.value(0)
    pwm1.duty_u16(0)
    dir2.value(0)
    pwm2.duty_u16(0)

def forward():
    dir1.value(1)
    pwm1.duty_u16(32768)
    dir2.value(1)
    pwm2.duty_u16(32768)


sensor_buffer = {
    "front": [],
    "right": [],
    "left": []
    } #empty list to store the data, but now for all the sensors
buffer_size = 10 #how many measuremnets is measures at one time
correction_factor = {
    "front":0,
    "right":0,
    "left":0
    }#starts the correction factor at 0, this is an esitmate in how often the sensor is constianlty off, now for all 3 sensors
y=0.1#how fast it does correction
def cailbrated_distance(sensor,name):
    dist = sensor.get_distance()#this gets the distance measured from the tof sensor and asigns the value to the variable dist
    if dist is None or dist <=0:
        return None #changed from the previous version checks is sensor has failed and returned no object and invaild,better for debugging
    sensor_buffer[name].append(dist)#this is adding the dist variable to the end of our empty list
    if len(sensor_buffer[name]) > buffer_size: #if the number of variables is bigger than the buffer size
        sensor_buffer[name].pop(0)#this then removes the first variable(the oldest variable) in the sensor buffer list
    average_distance = sum(sensor_buffer[name]) /len(sensor_buffer[name])#sum adds all the variables, len counts how many variables there are
    return average_distance#this allows us to use the average value whenever the function is called

def fixed_cailbration_distance(name,sensor_measure, real_measure):
    global correction_factor#this is calling a variable that is not defined inside the function,hence global
    if sensor_measure is not None:
        error = sensor_measure - real_measure#if this was the final correction value it would make the correction jump around eg if one was 52 one 54, the correction factor would be 2 and 4
        correction_factor[name] = (1-y)*correction_factor[name]+y*error#this is a standard low pass filter to smooth out the correction
        return correction_factor[name]
    else:
        return correction_factor[name]
Front_wall = 100 #all these numbers need to be changed when measured for the maze
Right_wall = 100
count_target = 100
Left_wall = 100
while True:
    
    front_distance = cailbrated_distance(front,"front")
    right_distance = cailbrated_distance(right,"right")
    left_distance = cailbrated_distance(left,"left")
    
    if front_distance is not None:#checking it the front distance is an actual value
        front_distance -= fixed_cailbration_distance("front",front_distance, Front_wall) #what -= means is front_distance - fixed cailbration
    if right_distance is not None:
        right_distance -= fixed_cailbration_distance("right",right_distance, Right_wall)
    if left_distance is not None:
        left_distance -= fixed_cailbration_distance("left",left_distance, Left_wall)
    
    

   


