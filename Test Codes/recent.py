from machine import I2C, SoftI2C, Pin, PWM
import time
import math

from vl53l0x import VL53L0X
from vl6180x import VL6180X

# ================== SETTINGS ==================

MAX_PWM = 65535
MIN_PWM = 5000

TARGET_RPS = 35

FORWARD_TARGET_RPS = 25

PID_INTERVAL = 0.005

# Need to tune for R only
Kp = 400
Ki = 20
Kd = 1

CENTER_GAIN = 50          # steering strength
MAX_STEER = 500

RIGHT_WALL_TARGET = 37    # mm, desired distance from the right wall
 
# histories / flags (not heavily used now but kept for completeness)
Left_TOF_History = []
Right_TOF_History = []
Forward_TOF_History = []
left_miss_active = False
right_turn_active = False

Left_Missed_Count = 0
Right_Missed_Count = 0

previous_left = None
previous_right = None

TURN_SPEED = 15000
TURN_SPEED_SCALE = 1000

Circumference = (math.pi * 40)    # wheel circumference (if diameter is 40mm)
COUNTS_PER_REV = 60
COUNTS_PER_MM = (COUNTS_PER_REV / Circumference)

TURN_SCALE_LEFT  = 1.765
TURN_SCALE_LEFT_OPEN = 1.755
TURN_SCALE_LEFT_CLOSED = 1.765

TURN_SCALE_RIGHT = 1.66
TURN_SCALE_RIGHT_OPEN = 1.65
TURN_SCALE_RIGHT_CLOSED = 1.66

MIN_FRONT_DIST_NORMAL = 60      # mm: if front distance below this, handle front wall
MIN_FRONT_DIST_TURN = 45
MIN_FRONT_DIST = MIN_FRONT_DIST_NORMAL

right_trigger_active = False

MIN_INCREASE   = 25      # mm: sudden increase in side distance → gap / corner
WALL_THRESHOLD = 60      # mm: above this = "open" or gap

MIN_CHANGE = 1

MIN_SIDE_DIST = 18

# ================== MOTOR SETUP ==================

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
    speed = speed * 1.035
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

# ================== ENCODERS ==================

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

Left_Encoder_A.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=Left_Encoder)
Right_Encoder_A.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=Right_Encoder)

# ================== TURN FUNCTIONS ==================

def Turn_Left_90():
    global Left_Encoder_Count, Right_Encoder_Count
    
    Drive_Forward_mm(15)

    TURN_MM = (math.pi * 95.0) / 4.0
    TARGET_COUNTS = int(TURN_MM * COUNTS_PER_MM * TURN_SCALE_LEFT)

    start_L = Left_Encoder_Count
    start_R = Right_Encoder_Count

    while True:
        dL = abs(Left_Encoder_Count - start_L)
        dR = abs(Right_Encoder_Count - start_R)

        if dL >= TARGET_COUNTS or dR >= TARGET_COUNTS:
            break

        Left_Motor_Set(-TURN_SPEED)
        Right_Motor_Set(TURN_SPEED)

    Motor_Stop()
    Reset_State()
    time.sleep(0.5)

def Turn_Right_90():
    global Left_Encoder_Count, Right_Encoder_Count
    
    Drive_Forward_mm(15)
    
    TURN_MM = (math.pi * 95.0) / 4.0
    TARGET_COUNTS = int(TURN_MM * COUNTS_PER_MM * TURN_SCALE_RIGHT)

    start_L = Left_Encoder_Count
    start_R = Right_Encoder_Count

    while True:
        dL = abs(Left_Encoder_Count - start_L)
        dR = abs(Right_Encoder_Count - start_R)

        if dL >= TARGET_COUNTS or dR >= TARGET_COUNTS:
            break

        Left_Motor_Set(TURN_SPEED - TURN_SPEED_SCALE)
        Right_Motor_Set(-TURN_SPEED + TURN_SPEED_SCALE)

    Motor_Stop()
    Reset_State()
    time.sleep(0.5)

# ================== PID CONTROLLER ==================

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

        return int(self.Kp * error + self.Ki * self.integral + self.Kd * derivative)

Left_PID  = PID(Kp, Ki, Kd, integral_limit=10000)
Right_PID = PID(Kp, Ki, Kd, integral_limit=10000)

# PID speed measurement history (globals used by PID_Drive & Reset_State)
last_L = 0
last_R = 0
last_time = time.ticks_ms()

def Reset_State():
    """
    Reset detection history and PID-related state after big movements
    (turns, wall sequences, etc.).
    """
    global previous_left, previous_right, last_L, last_R, last_time

    # reset wall detection history
    previous_left = None
    previous_right = None

    # reset PID internal state
    Left_PID.integral = 0.0
    Left_PID.last_error = 0.0
    Right_PID.integral = 0.0
    Right_PID.last_error = 0.0

    # reset speed measurement baseline
    last_L = Left_Encoder_Count
    last_R = Right_Encoder_Count
    last_time = time.ticks_ms()

# ================== ToF SETUP ==================

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

# ================== PID DRIVE: RIGHT-WALL FOLLOWING ==================

def PID_Drive(right_mm):
    """
    Keeps wheel speeds near TARGET_RPS and uses ONLY the right ToF
    sensor to follow the right wall at RIGHT_WALL_TARGET distance (35mm).
    """
    global last_L, last_R, last_time

    now = time.ticks_ms()
    dt = time.ticks_diff(now, last_time) / 1000.0
    if dt < PID_INTERVAL:
        return
    last_time = now

    # how many encoder counts since last update
    Left_Counts = Left_Encoder_Count - last_L
    Right_Counts = Right_Encoder_Count - last_R
    last_L = Left_Encoder_Count
    last_R = Right_Encoder_Count

    # basic wheel speed control (keep both wheels near TARGET_RPS)
    Left_PWM = Left_PID.compute(TARGET_RPS, Left_Counts, dt)
    Right_PWM = Right_PID.compute(TARGET_RPS, Right_Counts, dt)

    # apply base speed
    Left_Motor_Set(Left_PWM)
    Right_Motor_Set(Right_PWM)

    # ==== RIGHT-WALL FOLLOWING STEERING (right ToF only) ====
    # Only steer when we actually have a wall in a sensible range.
    # If right_mm is very large (>= WALL_THRESHOLD) we treat it as "gap / no wall"
    # and DO NOT steer into it.
    if 0 < right_mm < WALL_THRESHOLD:
        # error > 0 → too far from right wall (want to move closer)
        error = right_mm - RIGHT_WALL_TARGET

        steering = CENTER_GAIN * error
        steering = _clamp(steering, -MAX_STEER, MAX_STEER)

        # Too far from right wall (error>0) → turn RIGHT:
        #   speed up left, slow down right.
        new_left_pwm  = _clamp(Left_PWM  + steering, -MAX_PWM, MAX_PWM)
        new_right_pwm = _clamp(Right_PWM - steering, -MAX_PWM, MAX_PWM)

        Left_Motor_Set(new_left_pwm)
        Right_Motor_Set(new_right_pwm)
        time.sleep(0.01)
    else:
        # In a gap / no-wall region: keep going straight with base PWM
        # (no extra steering correction)
        pass


# ================== DRIVE FORWARD WITH FRONT SAFETY ==================

def Drive_Forward_mm(dist_mm):
    global Left_Encoder_Count, Right_Encoder_Count, MIN_FRONT_DIST
    
    start_L = Left_Encoder_Count
    start_R = Right_Encoder_Count
    Target_Counts = dist_mm * COUNTS_PER_MM
    
    last_count_L = Left_Encoder_Count
    last_count_R = Right_Encoder_Count
    last_t = time.ticks_ms()
    
    while True:
        dL_total = abs(Left_Encoder_Count - start_L)
        dR_total = abs(Right_Encoder_Count - start_R)
        avg_counts = (dL_total + dR_total) / 2.0
        if avg_counts >= Target_Counts:
            break
        
        front_mm = Read_Front_mm()
        if 0 < front_mm <= MIN_FRONT_DIST:
            break
        
        now = time.ticks_ms()
        dt = time.ticks_diff(now, last_t) / 1000.0
        if dt <= 0:
            dt = 1e-6
            
        if dt < PID_INTERVAL:
            time.sleep(0.002)
            continue
        
        delta_L = Left_Encoder_Count - last_count_L
        delta_R = Right_Encoder_Count - last_count_R

        last_count_L = Left_Encoder_Count
        last_count_R = Right_Encoder_Count
        last_t = now
        
        measured_rps_L = (delta_L / COUNTS_PER_REV) / dt
        measured_rps_R = (delta_R / COUNTS_PER_REV) / dt
        
        Left_PWM = Left_PID.compute(FORWARD_TARGET_RPS, measured_rps_L, dt)
        Right_PWM = Right_PID.compute(FORWARD_TARGET_RPS, measured_rps_R, dt)
        
        Left_PWM = int(round(Left_PWM * 1.0))
        Right_PWM = int(round(Right_PWM * 1.02))
        
        Left_PWM = _clamp(Left_PWM, 0, MAX_PWM)
        Right_PWM = _clamp(Right_PWM, 0, MAX_PWM)
        
        Left_Motor_Set(Left_PWM)
        Right_Motor_Set(Right_PWM)
        
    Motor_Stop()
    Reset_State()
    time.sleep(0.05)

def Drive_Forward_mm_Crashable(dist_mm):
    global Left_Encoder_Count, Right_Encoder_Count, MIN_FRONT_DIST
    
    start_L = Left_Encoder_Count
    start_R = Right_Encoder_Count
    Target_Counts = dist_mm * COUNTS_PER_MM
    
    last_count_L = Left_Encoder_Count
    last_count_R = Right_Encoder_Count
    last_t = time.ticks_ms()
    
    while True:
        dL_total = abs(Left_Encoder_Count - start_L)
        dR_total = abs(Right_Encoder_Count - start_R)
        avg_counts = (dL_total + dR_total) / 2.0
        if avg_counts >= Target_Counts:
            break
        
        now = time.ticks_ms()
        dt = time.ticks_diff(now, last_t) / 1000.0
        if dt <= 0:
            dt = 1e-6
            
        if dt < PID_INTERVAL:
            time.sleep(0.002)
            continue
        
        delta_L = Left_Encoder_Count - last_count_L
        delta_R = Right_Encoder_Count - last_count_R

        last_count_L = Left_Encoder_Count
        last_count_R = Right_Encoder_Count
        last_t = now
        
        measured_rps_L = (delta_L / COUNTS_PER_REV) / dt
        measured_rps_R = (delta_R / COUNTS_PER_REV) / dt
        
        Left_PWM = Left_PID.compute(FORWARD_TARGET_RPS, measured_rps_L, dt)
        Right_PWM = Right_PID.compute(FORWARD_TARGET_RPS, measured_rps_R, dt)
        
        Left_PWM = int(round(Left_PWM * 1.0))
        Right_PWM = int(round(Right_PWM * 1.02))
        
        Left_PWM = _clamp(Left_PWM, 0, MAX_PWM)
        Right_PWM = _clamp(Right_PWM, 0, MAX_PWM)
        
        Left_Motor_Set(Left_PWM)
        Right_Motor_Set(Right_PWM)
        
    Motor_Stop()
    Reset_State()
    time.sleep(0.05)

# ================== WALL TURN SEQUENCES ==================

def Wall_Turn_Right():
    Motor_Stop()
    time.sleep(0.25)
    
    Drive_Forward_mm(50)
    time.sleep(0.25)
    
    Left_Close()
    Right_Close()
    time.sleep(0.25)
        
    Drive_Forward_mm(50)
    time.sleep(0.25)
    
    Left_Close()
    Right_Close()
    time.sleep(0.25)
        
    Drive_Forward_mm(50)
    time.sleep(0.25)
    
    Left_Close()
    Right_Close()
    time.sleep(0.25)
        
    Drive_Forward_mm(30)
    time.sleep(0.25)
    
    Left_Close()
    Right_Close()
    time.sleep(0.25)
    
    Motor_Stop()
    time.sleep(0.25)
    
    Drive_Forward_mm_Crashable(20)
    time.sleep(0.25)
    
    Reset_State()
    time.sleep(0.25)
    
    Turn_Right_90()
    time.sleep(1.0)
    
    Motor_Stop()
    time.sleep(0.25)
    
    Drive_Forward_mm(50)
    time.sleep(0.25)
    
    Left_Close()
    Right_Close()
    time.sleep(0.25)
    
    Drive_Forward_mm(50)
    time.sleep(0.25)
    
    Left_Close()
    Right_Close()
    time.sleep(0.25)
    
    Drive_Forward_mm(50)
    time.sleep(0.25)
    
    Left_Close()
    Right_Close()
    time.sleep(0.25)
    
    Drive_Forward_mm(50)
    time.sleep(0.25)
    
    Left_Close()
    Right_Close()
    time.sleep(0.25)
    
    Drive_Forward_mm_Crashable(20)
    time.sleep(0.25)
    
    Motor_Stop()
    time.sleep(0.25)

    # reset state after this complex movement
    Reset_State()


# ================== FRONT WALL HANDLING ==================

def Handle_Front_Wall(front_mm):
    global previous_left, previous_right, TURN_SCALE_LEFT, TURN_SCALE_RIGHT

    Motor_Stop()
    time.sleep(0.1)

    left_mm  = Read_Left_mm()
    right_mm = Read_Right_mm()

    RIGHT_OPEN = right_mm > WALL_THRESHOLD
    LEFT_OPEN  = left_mm  > WALL_THRESHOLD

    if RIGHT_OPEN:
        TURN_SCALE_RIGHT = TURN_SCALE_RIGHT_OPEN
        Motor_Stop()
        time.sleep(0.25)
        Drive_Forward_mm_Crashable(10)
        time.sleep(0.25)
        Turn_Right_90()
        time.sleep(0.5)
        Drive_Forward_mm(130)
        TURN_SCALE_RIGHT = TURN_SCALE_RIGHT_CLOSED
        time.sleep(0.25)
    elif LEFT_OPEN:
        TURN_SCALE_LEFT = TURN_SCALE_LEFT_OPEN
        Motor_Stop()
        time.sleep(0.25)
        Drive_Forward_mm_Crashable(10)
        time.sleep(0.25)
        Turn_Left_90()
        time.sleep(0.5)
        Drive_Forward_mm(130)
        TURN_SCALE_LEFT = TURN_SCALE_LEFT_CLOSED
        time.sleep(0.25)
    else:
        # dead end: default to left turn
        Turn_Left_90()
        time.sleep(0.5)

    # after any big turn, reset detection & PID state
    Reset_State()
    return

# ================== WALL DETECTION (SIDE GAPS / CORNERS) ==================

def Detect_Wall(current, previous):
    """
    Detect a new opening / corner on the right.

    We deliberately DO NOT reject large 'current' values (like 180+),
    because a sudden jump to a large distance usually means the wall
    has disappeared (gap / corner), which is exactly what we want to
    detect.
    """
    if previous is None:
        # First valid reading: no detection yet, just store it.
        return False, current

    # Ignore clearly invalid / missing measurements
    if current <= 0:
        return False, current

    delta = current - previous

    # Sudden increase in distance → likely corner / opening
    corner = delta >= MIN_INCREASE
    # Distance larger than normal wall-follow range → treat as opening
    wall_appear = current >= WALL_THRESHOLD

    triggered = corner or wall_appear

    return triggered, current

def Detect_Wall_Decrease(current, previous):
    if previous is None:
        # First valid reading: no detection yet, just store it.
        return False, current

    # Ignore clearly invalid / missing measurements
    if current <= 0:
        return False, current

    delta = current - previous

    corner = delta >= MIN_CHANGE
    
    wall_close = current <= MIN_SIDE_DIST

    triggered = corner and wall_close

    return triggered, current

# ================== SMALL "STRAIGHTEN" TURNS ==================

def Left_Close():
    left_mm = Read_Left_mm()
    if 0 < left_mm <= 15:
        Right_Straight_Turn()
        time.sleep(0.25)
        Right_Straight_Turn()
        time.sleep(0.25)

def Right_Close():
    right_mm = Read_Right_mm()
    if 0 < right_mm <= 15:
        Left_Straight_Turn()
        time.sleep(0.25)
        Left_Straight_Turn()
        time.sleep(0.25)

def Left_Straight_Turn():
    # tiny left-turn (move away from right wall)
    Left_Motor_Set(-12500)
    Right_Motor_Set(12500)
    time.sleep(0.05)
    Motor_Stop()

def Right_Straight_Turn():
    # tiny right-turn (move away from left wall)
    Left_Motor_Set(12500)
    Right_Motor_Set(-12500)
    time.sleep(0.05)
    Motor_Stop()

def Left_Straight_Turn_Small():
    # tiny left-turn (move away from right wall)
    Left_Motor_Set(-12500)
    Right_Motor_Set(12500)
    time.sleep(0.025)
    Motor_Stop()

def Right_Straight_Turn_Small():
    # tiny right-turn (move away from left wall)
    Left_Motor_Set(12500)
    Right_Motor_Set(-12500)
    time.sleep(0.025)
    Motor_Stop()

# ================== MAIN LOOP ==================

try:
    Reset_State()  # initial reset
    
    while True:
        front_mm = Read_Front_mm()
        left_mm  = Read_Left_mm()
        right_mm = Read_Right_mm()
        
        raw_front_mm = front_mm
        raw_left_mm  = left_mm
        raw_right_mm = right_mm
        
        # Clamp readings to remove obvious nonsense
        # Use a higher max for right_mm so large 'gap' readings are preserved.
        right_mm = _clamp(right_mm, 15, 255)
        left_mm  = _clamp(left_mm, 15, 255)    # only used for safety checks
        front_mm = _clamp(front_mm, 20, 1200)
        
        # Right-wall gap / corner detection only (use the clamped right_mm above)
        right_triggered, previous_right = Detect_Wall(right_mm, previous_right)
        
        right_close_triggered, previous_right = Detect_Wall_Decrease(right_mm, previous_right)
        
        left_close_triggered, previous_left = Detect_Wall_Decrease(left_mm, previous_left)

        # 1) Front obstacle: handle turn and continue
        if 0 < front_mm <= MIN_FRONT_DIST:
            Motor_Stop()
            Handle_Front_Wall(front_mm)
            continue
        
        # 2) Emergency: too close to left wall
        if right_close_triggered:
            Left_Straight_Turn_Small()
            Reset_State()
            continue
            
        # Emergency: too close to right wall
        if left_close_triggered:
            Right_Straight_Turn_Small()
            Reset_State()
            continue
        
        # 2) Emergency: too close to left wall
        if raw_left_mm > 0 and raw_left_mm <= 10:
            Right_Straight_Turn()
            Drive_Forward_mm(2)
            Reset_State()
            continue
            
        # Emergency: too close to right wall
        if raw_right_mm > 0 and raw_right_mm <= 10:
            Left_Straight_Turn()
            Drive_Forward_mm(2)
            Reset_State()
            continue
            
        # 3) New opening / corner on the RIGHT: stop, reset, then do wall-turn
        if right_triggered:
            right_trigger_active = True
            MIN_FRONT_DIST = MIN_FRONT_DIST_TURN
            
            Motor_Stop()
            Reset_State()
            time.sleep(0.5)           
            Wall_Turn_Right()
            
            MIN_FRONT_DIST = MIN_FRONT_DIST_NORMAL
            right_trigger_active = False
            continue

        # 4) Normal right-wall following with PID around 35mm
        new_right_mm = Read_Right_mm()
        new_right_mm = _clamp(new_right_mm, 15, 180)
        
        PID_Drive(new_right_mm)
        time.sleep(0.01)

except KeyboardInterrupt:
    print("Stopped by user.")
finally:
    Motor_Stop()
    print("Shutdown complete")

# ================== End ==================
