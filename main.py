from machine import I2C, SoftI2C, Pin, PWM
import time

from vl53l0x import VL53L0X
from vl53I0x1 import VL6180X

i2c_front = I2C(1, scl=Pin(19), sda=Pin(18)) #sets up for all the tof, they all share one i2c

i2c_right = SoftI2C(scl=Pin(27), sda=Pin(26))

#i2c_left = I2C(1, scl=Pin(21), sda=Pin(20))

front = VL53L0X(i2c_front, address=0x29) #this is setting up the address for all the tof sensors, the front one must have address of 0x29
right = VL6180X(i2c_right, address=0x29)
#left = VL6180X(i2c_left, address=0x31)

#setting up the speed and direction pins for the motor, from the motor controller
dir1 = Pin(0,Pin.OUT) #left motor
dir2 = Pin(2,Pin.OUT) #right motor
pwm1 = PWM(Pin(1))
pwm2 = PWM(Pin(3))
pwm1.freq(1000)
pwm2.freq(1000)

#encoder pins
encoder_l_a=Pin(4,Pin.IN)
encoder_l_b=Pin(5,Pin.IN)
encoder_r_a=Pin(6,Pin.IN)
encoder_r_b=Pin(7,Pin.IN)

#this is a place to store the pulses of the encoders
encoder_countl=0
encoder_countr=0

#encoder interrput callback functions
def encoder_call_left(pin):#if the value of the encoder is 0 it will add a count to the encoder_count(moving forward) if not it will minus one, this works as the function is only called when there is a change in the pin, which must mean the motor is moving
    global encoder_countl
    if encoder_l_b.value() == 0:
        encoder_countl +=1
    else:
        encoder_countl -=1
    
def encoder_call_right(pin):
    global encoder_countr
    if encoder_r_b.value() == 0:
        encoder_countr +=1
    else:
        encoder_countr -=1
  
#interrputs for the encoders
encoder_l_a.irq(trigger=Pin.IRQ_RISING, handler=encoder_call_left) #whenever a pulse is detetecd from the encoder it runs the encoder call functions

encoder_r_a.irq(trigger=Pin.IRQ_RISING, handler=encoder_call_right)



def turn_left(count_target):
    global encoder_countl, encoder_countr
    encoder_countl=0
    encoder_countr=0
    
    dir1.value(0)
    pwm1.duty_u16(0)
    dir2.value(1)
    pwm2.duty_u16(32768)
    
    while encoder_countr< count_target: #waiting for this condition to become false
        time.sleep_ms(1)
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
        time.sleep_ms(1)
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
    #"left": []
    } #empty list to store the data, but now for all the sensors
buffer_size = 10 #how many measuremnets is measures at one time
correction_factor = {
    "front":0,
    "right":0,
    #"left":0
    }#starts the correction factor at 0, this is an esitmate in how often the sensor is constianlty off, now for all 3 sensors
y=0.1#how fast it does correction
def cailbrated_distance(sensor,name):
    if isinstance(sensor, VL6180X):
        dist = sensor.range()           
    else:
        dist = sensor.read()
    if dist is None or dist <=0:
        return None #changed from the previous version checks is sensor has failed and returned no object and invaild,better for debugging
    sensor_buffer[name].append(dist)#this is adding the dist variable to the end of our empty list
    if len(sensor_buffer[name]) > buffer_size: #if the number of variables is bigger than the buffer size
        sensor_buffer[name].pop(0)#this then removes the first variable(the oldest variable) in the sensor buffer list
    average_distance = sum(sensor_buffer[name]) /len(sensor_buffer[name])#sum adds all the variables, len counts how many variables there are
    return average_distance#this allows us to use the average value whenever the function is called

def fixed_cailbration_distance(name,sensor_measure):
    global correction_factor#this is calling a variable that is not defined inside the function,hence global
    if sensor_measure is not None:
        error = sensor_measure - correction_factor[name]#if this was the final correction value it would make the correction jump around eg if one was 52 one 54, the correction factor would be 2 and 4
        correction_factor[name] = (1-y)*correction_factor[name]+y*error#this is a standard low pass filter to smooth out the correction
        return correction_factor[name]
    else:
        return correction_factor[name]

def right_wall_following(front_distance, right_distance):
    global count_target
    if right_distance is not None and right_distance > safe_right:
        turn_right(count_target)
    elif front_distance is not None and front_distance > safe_front:
        forward()
    else:
        turn_left(count_target)
    
    


count_target = 15
safe_right = 40
safe_front = 50
while True:    
    front_distance = cailbrated_distance(front,"front")
    right_distance = cailbrated_distance(right,"right")
  #  left_distance = cailbrated_distance(left,"left")
    
    if front_distance is not None:#checking it the front distance is an actual value
        front_distance -= fixed_cailbration_distance("front",front_distance) #what -= means is front_distance - fixed cailbration
    if right_distance is not None:
        right_distance -= fixed_cailbration_distance("right",right_distance)
    #if left_distance is not None:
        #left_distance -= fixed_cailbration_distance("left",left_distance)
    
    right_wall_following(front_distance, right_distance)
    
    time.sleep_ms(20)