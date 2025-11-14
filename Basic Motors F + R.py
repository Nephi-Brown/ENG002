from machine import Pin, PWM, I2C
import time

# -- Motor Setup --

# - Left Motor -
DIR1 = Pin(0, Pin.OUT)
PWM1 = Pin(Pin(1))
PWM1.freq(20000)

def LMotor_Forward(speed):
    DIR1.value(1)
    PWM.duty_u16(speed)
    
def LMotor_Reverse(speed):
    DIR1.value(0)
    PWM.duty_u16(speed)
    
    def LMotor_Stop():
    PWM1.duty_u16(0)


# - Right Motor DIR/PWM -
DIR2 = Pin(2, Pin.OUT)
PWM2 = Pin(Pin(3))
PWM2.freq(20000)
    
def RMotor_Forward(speed):
    DIR2.value(1)
    PWM2.duty_u16(speed)

def RMotor_Reverse(speed):
    DIR2.value(0)
    PWM2.duty_u16(speed)
    
def RMotor_Stop():
    PWM2.duty_u16(0)
    



# - Speed Parameters -

AvSpd = 20000
MinSpd = 10000
MaxSpd = 30000




# -- Motor Encoder --

# - Left Motor Encoder - 
LEncA = Pin(4, Pin.IN, Pin.PULL_UP)
LEncB = Pin(5, Pin.IN, Pin.PULL_UP)

LEnc_Count = 0

LPrevA = LEncA.value()

def Left_Encoder(pin):
    global LEnc_Count, LPrevA
    A = LEncA.value()
    B = LEncB.value()
    if A != LPrevA;
        LEnc_Count += 1 if A == B else -1
        LPrevA = A

LEncA.irq(trigger=Pin.IRQ_RISING | Pin,IRQ_FALLING, handler=Right_Encoder)


# - Right Motor Encoder -
REncA = Pin(6, Pin.IN, Pin.PULL_UP)
REncB = Pin(7, Pin.IN, Pin.PULL_UP)

REnc_Count = 0

RPrevA = REncA.value()

def Right_Encoder(pin):
    global REnc_Count, RPrevA
    A = LEncA.value()
    B = REncB.value()
    if A != RPrevA;
        REnc_Count += 1 if A == B else -1
        RPrevA = A

REncA.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=Left_Encoder)






--------------------------------------------------
# Import ToF sensor setup config - Xshut or I2C???
--------------------------------------------------






# -- Run --

while True:
    print(Left Encoder:"LEnc_Count" Right Encoder:"REnc_Count")

# - Motors Spin -

# - Left forward 1 second, 20k speed
LMotor_Forward(20000)

print("LEnc_Count")

time.sleep(1)

# - Left stopped 1 second
LMotor_Stop()

print("LEnc_Count")

time.sleep(1)

# - Left reversed 1 second, 20k speed
LMotor_Reverse(20000)

print("LEnc_Count")

time.sleep(1)

# - Left stopped 1 second
LMotor_Stop()

print("LEnc_Count")

time.sleep(1)


# - switch motor - 


# - Right forward 1 second, 20k speed
RMotor_Forward(20000)

print("REnc_Count")

time.sleep(1)

# - Right stopped 1 second
RMotor_Stop()

print("REnc_Count")

time.sleep(1)

# - Right reversed 1 second, 20k speed
RMotor_Reverse(20000)

print("REnc_Count")

time.sleep(1)

# - Right stopped 1 second
RMotor_Stop()

print("REnc_Count")

time.sleep(1)


# - Half speed - 

# - Left forward 1 second, 10k speed
LMotor_Forward(10000)

print("LEnc_Count")

time.sleep(1)

# - Left stopped 1 second
LMotor_Stop()

print("LEnc_Count")

time.sleep(1)

# - Left reversed 1 second, 10k speed
LMotor_Reverse(10000)

print("LEnc_Count")

time.sleep(1)

# - Left stopped 1 second
LMotor_Stop()

print("LEnc_Count")

time.sleep(1)


# - switch motor - 


# - Right forward 1 second, 10k speed
RMotor_Forward(10000)

print("REnc_Count")

time.sleep(1)

# - Right stopped 1 second
RMotor_Stop()

print("REnc_Count")

time.sleep(1)

# - Right reversed 1 second, 10k speed
RMotor_Reverse(10000)

print("REnc_Count")

time.sleep(1)

# - Right stopped 1 second
RMotor_Stop()

print("REnc_Count")

time.sleep(1)


# -- Forever --
