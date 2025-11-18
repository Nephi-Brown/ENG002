from machine import I2C, SoftI2C, Pin, PWM
import time

from vl53l0x import VL53L0X
from vl6180x import VL6180X


# ===== Settings =====

# - PWM Settings -
MAX_PWM = 65535
Min_PWM = 10000
TARGET_RPS = 10

# - Centering Settings -
PID_Interval = 0.01
Center_Gain = 100
Max_Steer = 5000 

# - Turning Settings -
Turn_Time = 0.25
Turn_Speed = 15000
SIDE_DROP_FACTOR = 0.5

About_Turn_Count = 0

Left_Missed_Count = 0

Right_Missed_Count = 0

previous_left = None

previous_right = None

# - Encoder Settings -
COUNTS_PER_MM = 2.0

# - Sensor Settings -
Min_Front_Dist = 60
Min_Increase = 25
Wall_Threshold = 60

# - Drive Distance Settings -
Drive_Grid = 160
Drive_Far = 125
Drive_Long = 100
Drive_Turn_After = 87
Drive_Medium = 75
Drive_Short = 50
Drive_Turn_Start = 37
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
    speed = _clamp(speed, -MAX_PWM, MAX_PWM)
    if speed >= 0:
        DIR1.value(1)
        PWM1.duty_u16(_clamp(speed, 0, MAX_PWM))
    else:
        DIR1.value(0)
        PWM1.duty_u16(_clamp(-speed, 0, MAX_PWM))
        
# - Right Motor Control -
def Right_Motor_Set(speed):
    speed = int(speed)
    speed = _clamp(speed, -MAX_PWM, MAX_PWM)
    if speed >= 0:
        DIR2.value(1)
        PWM2.duty_u16(_clamp(speed, 0, MAX_PWM))
    else:
        DIR2.value(0)
        PWM2.duty_u16(_clamp(-speed, 0, MAX_PWM))
        
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

def About_Turn():
    global About_Turn_Count
    Left_Turn()
    time.sleep(0.1)
    Left_Turn()
    time.sleep(0.1)
    About_Turn_Count += 1
    

# ===== Encoder Setup =====

# - Left Motor Encoder -
Left_Encoder_A = Pin(4, Pin.IN, Pin.PULL_UP)
Left_Encoder_B = Pin(5, Pin.IN, Pin.PULL_UP)

# - Right Motor Encoder -
Right_Encoder_A = Pin(6, Pin.IN, Pin.PULL_UP)
Right_Encoder_B = Pin(7, Pin.IN, Pin.PULL_UP)

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
    global last_L, last_R, last_time
    
    now = time.ticks_ms()
    dt = time.ticks_diff(now, last_time) / 500.0
    
    if dt < PID_Interval:
        return
    
    last_time = now
    
    # - Encoder Counts -
    Left_Counts = Left_Encoder_Count - last_L
    Right_Counts = Right_Encoder_Count - last_R
    last_L = Left_Encoder_Count
    last_R = Right_Encoder_Count
    
    # - Base PID Output, speed -
    Left_PWM = Left_PID.compute(TARGET_RPS, Left_Counts, dt)
    Right_PWM = Right_PID.compute(TARGET_RPS, Right_Counts, dt)
    
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
        Right_Corrected = Right_PWM + steering
        
        Left_Corrected = _clamp(Left_Corrected, -MAX_PWM, MAX_PWM)
        Right_Corrected = _clamp(Right_Corrected, -MAX_PWM, MAX_PWM)
        
        Left_Motor_Set(Left_Corrected)
        Right_Motor_Set(Right_Corrected)


# ===== Drive Functions =====

def Drive_Forward_mm(dist_mm, front_safety_mm=50):
    global Left_Encoder_Count, Right_Encoder_Count
    
    start_L = Left_Encoder_Count = 0
    start_R = Right_Encoder_Count = 0
    Target_Counts = abs(dist_mm * COUNTS_PER_MM)
    
    direction = 1 if dist_mm >= 0 else -1
    
    while True:
        dL = abs(Left_Encoder_Count - start_L)
        dR = abs(Right_Encoder_Count - start_R)
        avg_counts = (dL + dR) / 2.0
        if avg_counts >= Target_Counts:
            break
        
        front_mm = Read_Front_mm()
        left_mm  = Read_Left_mm()
        right_mm = Read_Right_mm()
        
        if 0 < front_mm <= front_safety_mm:
            break
        
        Left_PWM = direction * Left_PID.compute(TARGET_RPS, dL, 0.01)
        Right_PWM = direction * Right_PID.compute(TARGET_RPS, dR, 0.01)
        
        Left_Motor_Set(Left_PWM)
        Right_Motor_Set(Right_PWM)
        
        time.sleep(0.01)
        
    Motor_Stop()
    
    
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


# ===== Finish Area =====
front_turn_history = []  # list of distances after front-caused turns

def Record_Front_Turn_Distance(d_mm):
    if d_mm <= 0:
        return
    front_turn_history.append(d_mm)
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
    PID_Drive_Step(half2)

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
    global previous_left, previous_right
    Left_Gap = 0
    Right_Gap = 0
    print("Front wall at", front_mm, "mm -> Right 90 sequence")
    Left_PID.integral = Right_PID.integral = 0
    Left_PID.last_error = Right_PID.last_error = 0
    
    while True:
        right_triggered, previous_right = has_increased(right_mm, previous_right)
        if right_triggered:
            print("Right wall detected:", right_mm)
            Right_Gap += 1
            
        left_triggered, previous_left = has_increased(left_mm, previous_left)
        if left_triggered:
            print("Left wall detected:", left_mm)
            Left_Gap += 1
            
        if Left_Gap == 0 and Right_Gap == 0:
            About_Turn()
            time.sleep(0.5)
            
        if Left_Gap == 1 and Right_Gap == 1:
            Right_Turn()
            Left_Gap -= 1
            Right_Gap -= 1
            time.sleep(0.5)
            
        if Left_Gap == 1:
            Left_Turn()
            Left_Gap -= 1
            time.sleep(0.5)
            
        if Right_Gap == 1:
            Right_Turn()
            Right_Gap -= 1
            time.sleep(0.5)
        
        time.sleep(0.1)
        
        new_front = Read_Front_mm()
        print("Front after turn:", new_front, "mm")
        Record_Front_Turn_Distance(new_front)
        
        if 0 < new_front <= Min_Front_Dist:
            continue
        else:
            break
        
    if Finish_Condition():
        print("Finish condition detected!")
        Finish_Sequence()
        
        
# ===== Left / Right Turning When Detected =====

# - Right Turn -
def Wall_Right_Turn():
    print("Turning Right")
    
    Drive_Forward_mm(Drive_Turn_Start)
    time.sleep(0.25)
    
    Motor_Stop()
    time.sleep(0.25)
    
    Right_Turn()
    time.sleep(0.50)
    
    Motor_Stop()
    time.sleep(0.25)
    
    Drive_Forward_mm(Drive_Turn_After)
    time.sleep(0.5)
    
    Motor_Stop()
    time.sleep(0.25)

    
# - Left Turn -
def Wall_Left_Turn():
    print("Turning Left")
    
    Drive_Forward_mm(Drive_Turn_Start)
    time.sleep(0.25)
    
    Motor_Stop()
    time.sleep(0.25)
    
    Left_Turn()
    time.sleep(0.50)
    
    Motor_Stop()
    time.sleep(0.25)
    
    Drive_Forward_mm(Drive_Turn_After)
    time.sleep(0.5)
    
    Motor_Stop()
    time.sleep(0.25)
    
    
# ===== Wall Detection =====

def has_increased(current_value, previous_value_ref):
    if previous_value_ref is None:
        return False, current_value

    increased_enough = (current_value - previous_value_ref) >= Min_Increase
    above_threshold = current_value > Wall_Threshold

    triggered = increased_enough or above_threshold
    return triggered, current_value


# ===== Main =====

Startup_Value = 1

print("Starting Maze Solving Robot")

try:
    while True:
        front_mm = Read_Front_mm()
        left_mm  = Read_Left_mm()
        right_mm = Read_Right_mm()
        
        left_triggered = False
        right_triggered = False
        
        if Startup_Value == 1:
            Drive_Forward_mm(2)
            time.sleep(1.0)
            Motor_Stop()
            time.sleep(1.0)
            Drive_Forward_mm(-2)
            Startup_Value -= 1
            time.sleep(1.0)
            Motor_Stop()
            time.sleep(0.1)
            continue
        
        if 0 < front_mm <= Min_Front_Dist:
            Motor_Stop()
            Handle_Front_Wall(front_mm)
            continue
        
        if 0 < left_mm <= Wall_Threshold and 0 < front_mm <= Wall_Threshold and 0 < front_mm <= Min_Front_Dist:
            Motor_Stop()
            Handle_About_Turn(front_mm)
            continue

        if About_Turn_Count == 0:
            left_triggered, previous_left = has_increased(left_mm, previous_left)
            if left_triggered:
                Left_Missed_Count += 1
            
            right_triggered, previous_right = has_increased(right_mm, previous_right)
            if right_triggered:
                Right_Missed_Count += 1
                
        else:
            if About_Turn_Count == 1:
                if Left_Missed_Count >= 1:
                    right_triggered, previous_right = has_increased(right_mm, previous_right)
                    if right_triggered:
                        About_Turn_Count -= 1
                        print("Right wall detected:", right_mm)
                        Right_Missed_Count -= 1
                        Motor_Stop()
                        time.sleep(0.5)
                        Wall_Left_Turn()
                        time.sleep(2.0)
            
                if Right_Missed_Count >= 1:
                    left_triggered, previous_left = has_increased(left_mm, previous_left)
                    if left_triggered:
                        About_Turn_Count -= 1
                        print("Left wall detected:", left_mm)
                        Left_Missed_Count -= 1
                        Motor_Stop()
                        time.sleep(0.5)
                        Wall_Right_Turn()
                        time.sleep(2.0)
        
        if not left_triggered and not right_triggered:
            PID_Drive_Step(left_mm, right_mm)
            
        time.sleep(0.01)
        
except KeyboardInterrupt:
    print("Stopped by user.")

finally:
    Motor_Stop()
    print("Shutdown complete")
        
# ===== END =====
        

        


