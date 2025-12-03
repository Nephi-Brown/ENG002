from machine import Pin, PWM, I2C
import time

# ==========================
# Motor Setup
# ==========================

# Left Motor
DIR1 = Pin(0, Pin.OUT)
PWM1 = PWM(Pin(1))
PWM1.freq(20000)

def LMotor_Forward(speed):
    DIR1.value(0)
    PWM1.duty_u16(speed)

def LMotor_Reverse(speed):
    DIR1.value(1)
    PWM1.duty_u16(speed)

def LMotor_Stop():
    PWM1.duty_u16(0)


# Right Motor
DIR2 = Pin(2, Pin.OUT)
PWM2 = PWM(Pin(3))
PWM2.freq(20000)

def RMotor_Forward(speed):
    DIR2.value(0)
    PWM2.duty_u16(speed)

def RMotor_Reverse(speed):
    DIR2.value(1)
    PWM2.duty_u16(speed)

def RMotor_Stop():
    PWM2.duty_u16(0)


# ==========================
# Speed Parameters
# ==========================

AvSpd  = 22500
MinSpd = 15000
MaxSpd = 30000


# ==========================
# Motor Encoders
# ==========================

# Left Motor Encoder
LEncA = Pin(4, Pin.IN, Pin.PULL_UP)
LEncB = Pin(5, Pin.IN, Pin.PULL_UP)

LEnc_Count = 0
LPrevA = LEncA.value()

def Left_Encoder(pin):
    global LEnc_Count, LPrevA
    A = LEncA.value()
    B = LEncB.value()
    if A != LPrevA:
        LEnc_Count += 1 if A == B else -1
        LPrevA = A


# Right Motor Encoder
REncA = Pin(6, Pin.IN, Pin.PULL_UP)
REncB = Pin(7, Pin.IN, Pin.PULL_UP)

REnc_Count = 0
RPrevA = REncA.value()

def Right_Encoder(pin):
    global REnc_Count, RPrevA
    A = REncA.value()
    B = REncB.value()
    if A != RPrevA:
        REnc_Count += 1 if A == B else -1
        RPrevA = A


# Attach interrupts
LEncA.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=Left_Encoder)
REncA.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=Right_Encoder)


# ==========================
# ToF Sensor Placeholder
# ==========================

# --------------------------------------------------
# Import ToF sensor setup config - Xshut or I2C???
# --------------------------------------------------


# ==========================
# Helper for printing encoder counts
# ==========================

def print_counts(label=""):
    if label:
        print(label)
    print("Left Encoder:", LEnc_Count, " Right Encoder:", REnc_Count)
    print("-----------------------------")


# ==========================
# Motor Test Sequence
# ==========================

def motor_test():
    # ---- Full speed (20k) ----
    # Left forward 1 second
    LMotor_Forward(20000)
    time.sleep(1)
    LMotor_Stop()
    print_counts("Left forward 20k, 1s")

    # Left reverse 1 second
    LMotor_Reverse(20000)
    time.sleep(1)
    LMotor_Stop()
    print_counts("Left reverse 20k, 1s")

    # Right forward 1 second
    RMotor_Forward(20000)
    time.sleep(1)
    RMotor_Stop()
    print_counts("Right forward 20k, 1s")

    # Right reverse 1 second
    RMotor_Reverse(20000)
    time.sleep(1)
    RMotor_Stop()
    print_counts("Right reverse 20k, 1s")

    # ---- Half speed (10k) ----
    # Left forward 1 second
    LMotor_Forward(10000)
    time.sleep(1)
    LMotor_Stop()
    print_counts("Left forward 10k, 1s")

    # Left reverse 1 second
    LMotor_Reverse(10000)
    time.sleep(1)
    LMotor_Stop()
    print_counts("Left reverse 10k, 1s")

    # Right forward 1 second
    RMotor_Forward(10000)
    time.sleep(1)
    RMotor_Stop()
    print_counts("Right forward 10k, 1s")

    # Right reverse 1 second
    RMotor_Reverse(10000)
    time.sleep(1)
    RMotor_Stop()
    print_counts("Right reverse 10k, 1s")


# ==========================
# Main
# ==========================

# Run the test once, then keep printing counts
#motor_test()

while True:
    LMotor_Forward(MinSpd)
    RMotor_Forward(MinSpd)
    print_counts("Idle loop")
    time.sleep(0.5)
