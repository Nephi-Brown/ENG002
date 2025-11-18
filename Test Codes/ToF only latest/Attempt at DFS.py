from machine import I2C, SoftI2C, Pin, PWM
import time

from vl53l0x import VL53L0X
from vl6180x import VL6180X


# ===== Settings =====

# - PWM Settings -
Max_PWM = 65535
Min_PWM = 10000
Target_RPS = 15

# - Centering Settings -
PID_Interval = 0.05
Center_Gain = 150
Max_Steer = 5000 

# - Turning Settings -
Turn_Time = 0.4
Turn_Speed = 10000

# - Encoder Settings -
Counts_Per_MM = 2.0

# - Front Sensor Settings -
Min_Front_Dist = 60

# - Drive Distance Settings -
Drive_Grid = 160
Drive_Far = 125
Drive_Long = 100
Drive_Medium = 75
Drive_Short = 50
Drive_Micro = 25
Drive_Nano = 1

# - Finish Detection -
Finish_Tolerance = 0.1


# ===== Motor Setup =====

# - Left Motor -
DIR1 = Pin(0, Pin.OUT)
PWM1 = PWM(Pin(1)); PWM1.freq(20000)

# - Right Motor -
DIR2 = Pin(2, Pin.OUT)
PWM2 = PWM(Pin(3)); PWM2.freq(20000)

def _clamp(val, lo, hi):
    return lo if val < lo else (hi if val > hi else val)

# - Left Motor Control -
def Left_Motor_Set(speed):
    speed = int(speed)
    speed = _clamp(speed, -Max_PWM, Max_PWM)
    if speed <= 0:
        DIR1.value(1)
        PWM1.duty_u16(_clamp(speed))
    else:
        DIR1.value(0)
        PWM1.duty_u16(_clamp(-speed))
        
# - Right Motor Control -
def Right_Motor_Set(speed):
    speed = int(speed)
    speed = _clamp(speed, -Max_PWM, Max_PWM)
    if speed <= 0:
        DIR2.value(1)
        PWM2.duty_u16(_clamp(speed))
    else:
        DIR2.value(0)
        PWM2.duty_u16(_clamp(-speed))
        
# - Motor Stop -
def Motor_Stop():
    Left_Motor_Set(0)
    Right_Motor_Set(0)
    

# - Turning -
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
    
About_Turn_Count = 0

def About_Turn():
    global About_Turn_Count
    Left_Turn()
    time.sleep(0.1)
    Left_Turn()
    time.sleep(0.1)
    About_Turn_Count += 1
    

# ===== Encoder Setup =====

# - Left Motor Encoder -
Left_Encoder_A = Pin(12, Pin.IN, Pin.PULL_UP)
Left_Encoder_B = Pin(13, Pin.IN, Pin.PULL_UP)

# - Right Motor Encoder -
Right_Encoder_A = Pin(14, Pin.IN, Pin.PULL_UP)
Right_Encoder_B = Pin(15, Pin.IN, Pin.PULL_UP)

# - Encoder Counters -
Left_Encoder_Count = 0
Left_Previous_Count = Left_Encoder_A.value()

Right_Encoder_Count = 0
Right_Previous_Count = Right_Encoder_A.value()

# - Left Encoder Control - 
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
        
# - Increase / Decrease Encoder Values - 
Left_Encoder_A.irq(trigger = Pin.IRQ_RISING | Pin.IRQ_FALLING, handler = Left_Encoder)

Right_Encoder_A.irq(trigger = Pin.IRQ_RISING | Pin.IRQ_FALLING, handler = Right_Encoder)


# ===== PID Setup =====

class PID:
    def __init__(self, Kp, Ki, Kd, integral_limit=None):
        self.Kp = Kp; self.Ki = Ki; self.Kd = Kd
        self.integral = 0.0
        self.last_error = 0.0
        self.integral_limit = integral_limit
        
    def compute(self, target, measurement, dt):
        if dt <= 0:
            dt = 1e-6
        error = target - measurement
        self.integral += error * dt
        if self.integral_limit:
            self.integral = _clamp(self.integral,
                                   -self.integral_limit,
                                   self.integral_limit)
        derivative = (error - self.last_error) / dt
        self.last_error = error
        return int(self.Kp * error +
                   self.Ki * self.integral +
                   self.Kd * derivative) 

# - Set Left / Right PID -
Left_PID = PID(2000, 50, 0, integral_limit = 10000)
Right_PID = PID(2000, 50, 0, integral_limit = 10000)

# ===== ToF Setup =====

# - Front ToF -
def setup_vl53l0x_on_bus(i2c, label):
    for addr in i2c.scan():
        try:
            s = VL53L0X(i2c, address=addr)
            print(label, "VL53L0X at", hex(addr))
            return s
        except:
            pass
    return None

# - Left / Right ToF Setup -
def setup_vl6180x_on_bus(i2c, label):
    for addr in i2c.scan():
        try:
            s = VL6180X(i2c, address=addr)
            print(label, "VL6180X at", hex(addr))
            return s
        except:
            pass
    return None


# ===== I2C / Soft I2C Setup =====

# - Find ToF Sensors -
i2c_forward = I2C(1, sda=Pin(18), scl=Pin(19), freq=400000)
i2c_left    = SoftI2C(sda=Pin(20), scl=Pin(21), freq=400000)
i2c_right   = SoftI2C(sda=Pin(26), scl=Pin(27), freq=400000)

# - Set ToF Sensor Name -
tof_forward = setup_vl53l0x_on_bus(i2c_forward, "forward")
tof_left    = setup_vl6180x_on_bus(i2c_left,  "left")
tof_right   = setup_vl6180x_on_bus(i2c_right, "right")

# - Start ToF -
if tof_forward:
    try: tof_forward.start_continuous()
    except: pass
if tof_left:
    try: tof_left.start_range_continuous(50)
    except: pass
if tof_right:
    try: tof_right.start_range_continuous(50)
    except: pass
    
# - ToF Limits -
def Read_Front_mm():
    if not tof_forward:
        return -1
    try:
        return tof_forward.read_range()
    except:
        return -1


def Read_Left_mm():
    if not tof_left:
        return -1
    try:
        return tof_left.range
    except:
        return -1


def Read_Right_mm():
    if not tof_right:
        return -1
    try:
        return tof_right.range
    except:
        return -1


# ===== PID Drive Setup =====

last_L = last_R = 0
last_time = time.ticks_ms()

def PID_Drive_Step(left_mm, right_mm):
    global last_L, last_R, last_time, Left_Encoder_Count, Right_Encoder_Count
    
    now = time.ticks_ms()
    dt = time.ticks_diff(now, last_time) / 1000.0
    
    if dt < PID_Interval:
        return
    
    last_time = now
    
    # - Encoder Counts -
    Left_Counts = Left_Encoder_Count - last_L
    Right_Counts = Right_Encoder_Count - last_R
    last_L = Left_Encoder_Count
    last_R = Right_Encoder_Count
    
    # - Base PID Output, speed -
    Left_PWM = Left_PID.compute(Target_RPS, Left_Counts, dt)
    Right_PWM = Right_PID.compute(Target_RPS, Right_Counts, dt)
    
    # - Move -
    Left_Motor_Set(Left_PWM)
    Right_Motor_Set(Right_PWM)
    
    # - Centering -
    if left_mm > 0 and right_mm > 0:
        side_error = left_mm - right_mm
        Kp_Center = 80
        
        steering = Kp_Center * side_error
        
        if steering > 15000:
            steering = 15000
        if steering < -15000:
            steering = -15000
            
        Left_Corrected = Left_PWM - steering
        Right_Corrected = Right_PWM - steering
        
        Left_Corrected = _clamp(Left_Corrected, -Max_PWM, Max_PWM)
        Right_Corrected = _clamp(Right_Corrected, -Max_PWM, Max_PWM)
        
        Left_Motor_Set(Left_Corrected)
        Right_Motor_Set(Right_Corrected)
        

# ===== Drive Functions =====

# - Drive With Centering -
def Drive_Forward_Centering_mm(dist_mm):
    global Left_Encoder_Count, Right_Encoder_Count
    
    Start_L = Left_Encoder_Count
    Start_R = Right_Encoder_Count
    Target_Counts = dist_mm * Counts_Per_MM
    
    while True:
        dL = abs(Left_Encoder_Count - Start_L)
        dR = abs(Right_Encoder_Count - Start_R)
        Average_Counts = (dL + dR) / 2.0
        if Average_Counts >= Target_Counts:
            break
        
        front_mm = Read_Front_mm()
        left_mm  = Read_Left_mm()
        right_mm = Read_Right_mm()
        
        if 0 < front_mm <= Min_Front_Dist:
            break
        
        PID_Drive_Step(left_mm, right_mm)
        time.sleep(0.01)
        
    Motor_Stop()
    
# - Drive No Centering -
def PID_Drive_Step_No_Centering(dist_mm):
    global last_L, last_R, last_time, Left_Encoder_Count, Right_Encoder_Count
    
    now = time.ticks_ms()
    dt = time.ticks_diff(now, last_time) / 1000.0
    
    if dt < PID_Interval:
        return
    
    last_time = now
    
    # - Encoder Counts -
    Left_Counts = Left_Encoder_Count - last_L
    Right_Counts = Right_Encoder_Count - last_R
    last_L = Left_Encoder_Count
    last_R = Right_Encoder_Count
    
    Start_L = Left_Encoder_Count
    Start_R = Right_Encoder_Count
    Target_Counts = dist_mm * Counts_Per_MM
    
    # - Base PID Output, speed -
    Left_PWM = Left_PID.compute(Target_RPS, Left_Counts, dt)
    Right_PWM = Right_PID.compute(Target_RPS, Right_Counts, dt)
    
    # - Move -
    Left_Motor_Set(Left_PWM)
    Right_Motor_Set(Right_PWM)
    
    # - Read Front -
    initial_front_mm = Read_Front_mm()
    move_forward_to_mm = (initial_front_mm - dist_mm)
    
    while True:
        dL = abs(Left_Encoder_Count - Start_L)
        dR = abs(Right_Encoder_Count - Start_R)
        Average_Counts = (dL + dR) / 2.0
        if Average_Counts >= Target_Counts:
            break
        
        front_mm = Read_Front_mm()
        
        if 0 < front_mm <= Min_Front_Dist:
            break
        
        if front_mm >= move_forward_to_mm:
            time.sleep(0.01)
        else:
            time.sleep(0.01)
            break

    Motor_Stop()
    
# - Drive Until Xmm Front -    
def Drive_Until_Front(target1):
    while True:
        front_mm = Read_Front_mm()
        if 0 < front_mm <= target1:
            Motor_Stop()
        else:
            PID_Drive_Step_No_Centering(Drive_Nano)


# ===== Finish Area =====
front_turn_history = []  # list of distances after front-caused turns

def Record_Front_Turn_Distance(front_mm):
    if front_mm <= 0:
        return
    front_turn_history.append(front_mm)
    if len(front_turn_history) > 3:
        front_turn_history.pop(0)

def Finish_Condition():
    if len(front_turn_history) < 3:
        return False
    a, b, c = front_turn_history
    avg = (a + b + c) / 3.0
    if avg <= 0:
        return False

    def within(x):
        return abs(x - avg) <= Finish_Tolerance * avg

    return within(a) and within(b) and within(c)

def Finish_Sequence():
    """
    When finish is detected:
    - drive until front distance is half of last recorded distance
    - 90° left turn on the spot
    - read front again, drive half of that distance
    - spin left on the spot 3 times, then stop forever
    """
    if len(front_turn_history) == 0:
        return

    last_d = front_turn_history[-1]
    if last_d <= 0:
        last_d = 100  # fallback

    # 1) Drive until front is half that distance
    target1 = last_d / 2.0
    print("Finish: drive until front ~", target1, "mm")
    Drive_Until_Front(target1)

    # 2) Turn 90° left
    print("Finish: Turn 90 left")
    Left_Turn()
    time.sleep(0.1)

    # 3) New front distance
    d2 = Read_Front_mm()
    if d2 <= 0:
        d2 = 100
    half2 = d2 / 2.0

    # 4) Drive half of that
    print("Finish: drive", half2, "mm forward")
    PID_Drive_Step_No_Centering(half2)

    # 5) Spin left 3 times on the spot
    print("Finish: spin left 3x")
    for _ in range(12):  # 12 x 90° = 3 full rotations
        Left_Turn()
        time.sleep(0.05)

    Motor_Stop()
    print("Maze solved! Holding position.")
    while True:
        Motor_Stop()
        time.sleep(1)


# ===== Turns =====

# - Detect Front Wall -
def Handle_Front_Wall(front_mm):
    Left_PID.integral = Right_PID.integral = 0
    Left_PID.last_error = Right_PID.last_error = 0
    
    while True:
        About_Turn()
        
        new_front = Read_Front_mm()
        Record_Front_Turn_Distance(new_front)
        
        if 0 < new_front <= Min_Front_Dist:
            Left_Turn()
        else:
            break
        
    if Finish_Condition():
        Finish_Sequence()
        
        
# ===== Left / Right Turning When Detected =====

# - Right Turn -
Right_Count = 0

def Right_Turn_Detected():
    global Right_Count
    
    Right_Count += 1
    
    Motor_Stop()
    time.sleep(0.5)
    
    PID_Drive_Step_No_Centering(Drive_Short)
    
    Right_Turn()
    
    PID_Drive_Step_No_Centering(Drive_Short)
    
# - Left Turn -
Left_Count = 0

def Left_Turn_Detected():
    global Left_Count
    
    Left_Count += 1
    
    Motor_Stop()
    time.sleep(0.5)
    
    PID_Drive_Step_No_Centering(Drive_Short)
    
    Left_Turn()
    
    PID_Drive_Step_No_Centering(Drive_Short)
    
    
# ===== Main =====

print("Starting Maze Solving Robot")

Right_Turn_Count = 0
Left_Turn_Count = 0

try:
    while True:
        front_mm = Read_Front_mm()
        left_mm  = Read_Left_mm()
        right_mm = Read_Right_mm()
    
        if 0 < front_mm <= Min_Front_Dist:
            Motor_Stop()
            Handle_Front_Wall(front_mm)
            continue
        
        if About_Turn_Count >= 1:
            if Right_Turn_Count > 1:
                if (left_mm - 25) > 100:
                    Motor_Stop()
                    PID_Drive_Step_No_Centering(Drive_Short)
                    Left_Turn_Detected()
                    PID_Drive_Step_No_Centering(Drive_Micro)
                    Right_Turn_Count -= 1
                    About_Turn_Count -= 1
            elif Left_Turn_Count > 1:
                if (right_mm - 25) > 100:
                    Motor_Stop()
                    PID_Drive_Step_No_Centering(Drive_Short)
                    Right_Turn_Detected()
                    PID_Drive_Step_No_Centering(Drive_Micro)
                    Left_Turn_Count -= 1
                    About_Turn_Count -= 1
            else:
                PID_Drive_Step(left_mm, right_mm)
                        
        if (right_mm - 25) > 100:
            Right_Turn_Count += 1
            PID_Drive_Step_No_Centering(Drive_Short)
            if (left_mm - 25) > 100:
                Left_Turn_Count += 1
                PID_Drive_Step_No_Centering(Drive_Far)
            else:
                PID_Drive_Step_No_Centering(Drive_Far)
                if 0 < front_mm <= Min_Front_Dist:
                    PID_Drive_Step(left_mm, right_mm)
                    
        if (left_mm - 25) > 100:
            Left_Turn_Count += 1
            PID_Drive_Step_No_Centering(Drive_Short)
            if (right_mm - 25) > 100:
                Right_Turn_Count += 1
                PID_Drive_Step_No_Centering(Drive_Far)
            else:
                PID_Drive_Step_No_Centering(Drive_Far)
                if 0 < front_mm <= Min_Front_Dist:
                    PID_Drive_Step(left_mm, right_mm)
        else:
            PID_Drive_Step(left_mm, right_mm)
            time.sleep(0.01)
        
except KeyboardInterrupt:
    print("Stopped by user.")
        
finally:
    Motor_Stop()
    print("Shutdown complete")
        
# ===== END =====
        
        
        

        
