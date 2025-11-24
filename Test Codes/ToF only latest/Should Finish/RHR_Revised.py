from machine import I2C, SoftI2C, Pin, PWM
import time

from vl53l0x import VL53L0X
from vl6180x import VL6180X


# ===== Settings =====

MAX_PWM = 65535
MIN_PWM = 10000
TARGET_RPS = 15

PID_INTERVAL = 0.01
CENTER_GAIN = 150
MAX_STEER = 15000 

Turn_Time = 0.8
Turn_Speed = 10000

previous_left = None
previous_right = None

COUNTS_PER_MM = 2.0

Min_Front_Dist = 50
Min_Increase = 30
Wall_Threshold = 60

Drive_Grid = 160
Drive_Far = 125
Drive_Long = 100
Drive_Turn_After = 87
Drive_Medium = 75
Drive_Short = 50
Drive_Turn_Start = 37
Drive_Micro = 25
Drive_Nano = 1


# ===== Motor Setup =====

DIR1 = Pin(0, Pin.OUT)
PWM1 = PWM(Pin(1)); PWM1.freq(20000)

DIR2 = Pin(2, Pin.OUT)
PWM2 = PWM(Pin(3)); PWM2.freq(20000)

def _clamp(val, lo, hi):
    return lo if val < lo else (hi if val > hi else val)


def Left_Motor_Set(speed):
    speed = int(speed)
    speed = _clamp(speed, -MAX_PWM, MAX_PWM)
    
    if speed == 0:
        PWM1.duty_u16(0)
        DIR1.value(0)
        return
    
    if speed > 0:
        DIR1.value(1)
        duty = _clamp(speed, MIN_PWM, MAX_PWM)
        PWM1.duty_u16(duty)
    else:
        DIR1.value(0)
        duty = _clamp(-speed, MIN_PWM, MAX_PWM)
        PWM1.duty_u16(duty)
        
        
def Right_Motor_Set(speed):
    speed = int(speed)
    speed = _clamp(speed, -MAX_PWM, MAX_PWM)
    
    if speed == 0:
        PWM2.duty_u16(0)
        DIR2.value(0)
        return

    if speed > 0:
        DIR2.value(1)
        duty = _clamp(speed, MIN_PWM, MAX_PWM)
        PWM2.duty_u16(duty)
    else:
        DIR2.value(0)
        duty = _clamp(-speed, MIN_PWM, MAX_PWM)
        PWM2.duty_u16(duty)
        
        
def Motor_Stop():
    Left_Motor_Set(0)
    Right_Motor_Set(0)

def Left_Turn(speed=Turn_Speed, duration=Turn_Time):
    Left_Motor_Set(speed)
    Right_Motor_Set(-speed)
    time.sleep(duration)
    Motor_Stop()

def Right_Turn(speed=Turn_Speed, duration=Turn_Time):
    Left_Motor_Set(-speed)
    Right_Motor_Set(speed)
    time.sleep(duration)
    Motor_Stop()
    

# ===== Encoder Setup =====

Left_Encoder_A = Pin(4, Pin.IN, Pin.PULL_UP)
Left_Encoder_B = Pin(5, Pin.IN, Pin.PULL_UP)

Right_Encoder_A = Pin(6, Pin.IN, Pin.PULL_UP)
Right_Encoder_B = Pin(7, Pin.IN, Pin.PULL_UP)

Left_Encoder_Count = 0
Left_Previous_Count = Left_Encoder_A.value()

Right_Encoder_Count = 0
Right_Previous_Count = Right_Encoder_A.value()

def Left_Encoder(pin):
    global Left_Encoder_Count, Left_Previous_Count
    A = Left_Encoder_A.value(); B = Left_Encoder_B.value()
    if A != Left_Previous_Count:
        Left_Encoder_Count += 1 if A == B else -1
        Left_Previous_Count = A

def Right_Encoder(pin):
    global Right_Encoder_Count, Right_Previous_Count
    A = Right_Encoder_A.value(); B = Right_Encoder_B.value()
    if A != Right_Previous_Count:
        Right_Encoder_Count += -1 if A == B else 1
        Right_Previous_Count = A
        
Left_Encoder_A.irq(trigger = Pin.IRQ_RISING | Pin.IRQ_FALLING, handler = Left_Encoder)
Right_Encoder_A.irq(trigger = Pin.IRQ_RISING | Pin.IRQ_FALLING, handler = Right_Encoder)


# ===== PID =====

class PID:
    def __init__(self, Kp, Ki, Kd, integral_limit=None):
        self.Kp = Kp; self.Ki = Ki; self.Kd = Kd
        self.integral = 0.0
        self.last_error = 0.0
        self.integral_limit = integral_limit
        
    def compute(self, target_rps, measurement_counts, dt):
        if dt <= 0:
            dt = 1e-6

        measurement_rps = measurement_counts / dt
        error = target_rps - measurement_rps

        self.integral += error * dt
        if self.integral_limit:
            self.integral = _clamp(self.integral, -self.integral_limit, self.integral_limit)

        derivative = (error - self.last_error) / dt
        self.last_error = error

        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        return int(output)

Left_PID = PID(2000, 50, 0, integral_limit=10000)
Right_PID = PID(2000, 50, 0, integral_limit=10000)


# ===== ToF Setup =====

def setup_vl53l0x_on_bus(i2c, label):
    for addr in i2c.scan():
        try:
            s = VL53L0X(i2c, address=addr)
            print(label, "VL53L0X at", hex(addr))
            return s
        except:
            pass
    return None

def setup_vl6180x_on_bus(i2c, label):
    for addr in i2c.scan():
        try:
            s = VL6180X(i2c, address=addr)
            print(label, "VL6180X at", hex(addr))
            return s
        except:
            pass
    return None

i2c_forward = I2C(1, sda=Pin(18), scl=Pin(19), freq=400000)
i2c_left    = SoftI2C(sda=Pin(20), scl=Pin(21), freq=400000)
i2c_right   = SoftI2C(sda=Pin(26), scl=Pin(27), freq=400000)

tof_forward = setup_vl53l0x_on_bus(i2c_forward, "forward")
tof_left    = setup_vl6180x_on_bus(i2c_left, "left")
tof_right   = setup_vl6180x_on_bus(i2c_right, "right")

if tof_forward:
    try: tof_forward.start_continuous()
    except: pass
if tof_left:
    try: tof_left.start_range_continuous(50)
    except: pass
if tof_right:
    try: tof_right.start_range_continuous(50)
    except: pass

def Read_Front_mm():
    if not tof_forward: return -1
    try: return tof_forward.read_range()
    except: return -1

def Read_Left_mm():
    if not tof_left: return -1
    try: return tof_left.range
    except: return -1

def Read_Right_mm():
    if not tof_right: return -1
    try: return tof_right.range
    except: return -1


# ===== Corner / Wall Detection =====

def detect_corner_or_wall_change(current, previous):
    if previous is None:
        return False, current

    if current <= 0 or current > 200:
        return False, current
    
    delta = current - previous

    corner = delta >= Min_Increase
    no_wall = current >= Wall_Threshold

    triggered = corner or no_wall

    return triggered, current


# ===== PID Drive =====

last_L = last_R = 0
last_time = time.ticks_ms()

def PID_Drive_Step(left_mm, right_mm):
    global last_L, last_R, last_time
    
    now = time.ticks_ms()
    dt = time.ticks_diff(now, last_time) / 1000.0
    if dt < PID_INTERVAL: return
    last_time = now
    
    Left_Counts = Left_Encoder_Count - last_L
    Right_Counts = Right_Encoder_Count - last_R
    last_L = Left_Encoder_Count
    last_R = Right_Encoder_Count
    
    Left_PWM = Left_PID.compute(TARGET_RPS, Left_Counts, dt)
    Right_PWM = Right_PID.compute(TARGET_RPS, Right_Counts, dt)
    
    side_error = left_mm - right_mm
    steering = CENTER_GAIN * side_error
        
    steering = _clamp(steering, -MAX_STEER, MAX_STEER)
        
    Left_Motor_Set(_clamp(Left_PWM - steering, -MAX_PWM, MAX_PWM))
    Right_Motor_Set(_clamp(Right_PWM + steering, -MAX_PWM, MAX_PWM))


# ===== Drive Functions =====

# - Drive Forward Without Centering -
def Drive_Forward_mm(dist_mm, front_safety_mm=30):
    
    if dist_mm < 0:
        return Drive_Back_mm(-dist_mm)
    
    global Left_Encoder_Count, Right_Encoder_Count
    
    start_L = Left_Encoder_Count
    start_R = Right_Encoder_Count
    Target_Counts = dist_mm * COUNTS_PER_MM 
    
    prev_L = start_L
    prev_R = start_R
    prev_time = time.ticks_ms()    
    
    while True:
        dL = abs(Left_Encoder_Count - start_L)
        dR = abs(Right_Encoder_Count - start_R)
        avg_counts = (dL + dR) / 2.0
        if avg_counts >= Target_Counts:
            break
        
        front_mm = Read_Front_mm()
        if 0 < front_mm <= front_safety_mm:
            break
        
        now = time.ticks_ms()
        dt = time.ticks_diff(now, prev_time) / 1000.0
        if dt <= 0:
            dt = PID_INTERVAL
        prev_time = now
        
        delta_L = Left_Encoder_Count - prev_L
        delta_R = Right_Encoder_Count - prev_R
        prev_L = Left_Encoder_Count
        prev_R = Right_Encoder_Count

        Left_PWM = Left_PID.compute(TARGET_RPS, delta_L, dt)
        Right_PWM = Right_PID.compute(TARGET_RPS, delta_R, dt)

        Left_PWM = _clamp(Left_PWM, 0, MAX_PWM)
        Right_PWM = _clamp(Right_PWM, 0, MAX_PWM)

        Left_Motor_Set(Left_PWM)
        Right_Motor_Set(Right_PWM)

        time.sleep(PID_INTERVAL)

    Motor_Stop()
    
    
# - Drive Backwards Without Centering (Seems to be slower than forward for some reason, though this isnt often needed so it is fine as is) -
def Drive_Back_mm(dist_mm):
    
    if dist_mm < 0:
        return Drive_Forward_mm(-dist_mm)
    
    global Left_Encoder_Count, Right_Encoder_Count

    start_L = Left_Encoder_Count
    start_R = Right_Encoder_Count
    Target_Counts = dist_mm * COUNTS_PER_MM

    prev_L = start_L
    prev_R = start_R
    prev_time = time.ticks_ms()

    while True:
        dL = abs(Left_Encoder_Count - start_L)
        dR = abs(Right_Encoder_Count - start_R)
        avg_counts = (dL + dR) / 2.0
        if avg_counts >= Target_Counts:
            break

        now = time.ticks_ms()
        dt = time.ticks_diff(now, prev_time) / 1000.0
        if dt <= 0:
            dt = PID_INTERVAL
        prev_time = now

        delta_L = Left_Encoder_Count - prev_L
        delta_R = Right_Encoder_Count - prev_R
        prev_L = Left_Encoder_Count
        prev_R = Right_Encoder_Count
        
        Left_PWM = -Left_PID.compute(TARGET_RPS, delta_L, dt)
        Right_PWM = -Right_PID.compute(TARGET_RPS, delta_R, dt)

        Left_PWM = _clamp(Left_PWM, -MAX_PWM, 0)
        Right_PWM = _clamp(Right_PWM, -MAX_PWM, 0)

        Left_Motor_Set(Left_PWM)
        Right_Motor_Set(Right_PWM)

        time.sleep(PID_INTERVAL)

    Motor_Stop()


# - Drive Forward Until Front Distance To A Wall Is X mm -
def Drive_Until_Front(target_mm):
    while True:
        front_mm = Read_Front_mm()
        left_mm  = Read_Left_mm()
        right_mm = Read_Right_mm()

        if front_mm > 0 and front_mm <= target_mm:
            break

        PID_Drive_Step(left_mm, right_mm)
        time.sleep(0.01)

    Motor_Stop()


# ===== Turns =====
                            
def Handle_Front_Wall(front_mm):
    global previous_left, previous_right

    print("Front wall detected at", front_mm, "mm")

    Motor_Stop()
    time.sleep(0.1)

    Left_PID.integral = 0
    Right_PID.integral = 0
    Left_PID.last_error = 0
    Right_PID.last_error = 0

    left_mm  = Read_Left_mm()
    right_mm = Read_Right_mm()

    print("Side distances: L =", left_mm, "mm   R =", right_mm, "mm")

    RIGHT_OPEN = right_mm > Wall_Threshold
    LEFT_OPEN  = left_mm  > Wall_Threshold

    if RIGHT_OPEN:
        print("→ Turning RIGHT (right side open)")
        Right_Turn()
        time.sleep(0.5)

    elif LEFT_OPEN:
        print("→ Turning LEFT (left side open)")
        Left_Turn()
        time.sleep(0.5)

    else:
        print("→ Dead end → About Turn (180°)")
        Right_Turn()
        time.sleep(0.5)
        Right_Turn()
        time.sleep(0.5)

    return
        
        
# ===== Left / Right Turning When Detected =====
                                
def Wall_Right_Turn():
    print("Turning Right")
    
    Drive_Forward_mm(Drive_Turn_Start)
    time.sleep(0.5)
    
    Motor_Stop()
    time.sleep(0.5)
    
    Right_Turn()
    time.sleep(0.5)
    
    Motor_Stop()
    time.sleep(0.5)
    
    Drive_Forward_mm(Drive_Turn_After)
    time.sleep(0.5)
    
    Motor_Stop()
    time.sleep(0.5)

def Wall_Left_Turn():
    print("Turning Left")
    
    Drive_Forward_mm(Drive_Turn_Start)
    time.sleep(0.5)
    
    Motor_Stop()
    time.sleep(0.5)
    
    Left_Turn()
    time.sleep(0.5)
    
    Motor_Stop()
    time.sleep(0.5)
    
    Drive_Forward_mm(Drive_Turn_After)
    time.sleep(0.5)
    
    Motor_Stop()
    time.sleep(0.5)
    
       
# ===== Startup =====

def Startup_Sequence():
    Drive_Forward_mm(2)
    time.sleep(1.0)
    Motor_Stop()
    time.sleep(1.0)
        
    Drive_Back_mm(2)
    time.sleep(1.0)
    Motor_Stop()
    time.sleep(0.1)


# ===== Main =====

print("Starting Maze Solving Robot (Right Wall Following)")

Startup_Sequence()

try:
    while True:
        front_mm = Read_Front_mm()
        left_mm  = Read_Left_mm()
        right_mm = Read_Right_mm()
        
        if 0 < front_mm <= Min_Front_Dist:
            Motor_Stop()
            Handle_Front_Wall(front_mm)
            continue

        right_triggered, previous_right = detect_corner_or_wall_change(right_mm, previous_right)
        left_triggered, previous_left   = detect_corner_or_wall_change(left_mm, previous_left)

        if right_triggered:
            Right_Turn()
            continue
        
        if left_triggered:
            Drive_Forward_mm(140)
            continue
  
        PID_Drive_Step(left_mm, right_mm)
        time.sleep(0.01)

except KeyboardInterrupt:
    print("Stopped by user.")

finally:
    Motor_Stop()
    print("Shutdown complete")
        
# ===== END =====
        


