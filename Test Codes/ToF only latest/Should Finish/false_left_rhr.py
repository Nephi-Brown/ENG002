from machine import I2C, SoftI2C, Pin, PWM
import time
import math

from vl53l0x import VL53L0X
from vl6180x import VL6180X

# settings

MAX_PWM = 65535
MIN_PWM = 15000

TARGET_RPS = 20

PID_INTERVAL = 0.005

Kp = 2000
Ki = 50
Kd = 0

CENTER_GAIN = 100

# New: histories of left/right/forward ToF (most recent 4) and state flags
Left_TOF_History = []      # oldest at index 0
Right_TOF_History = []     # oldest at index 0
Forward_TOF_History = []   # oldest at index 0
left_miss_active = False   # "opening on left" handling active
right_turn_active = False  # special right-turn sequence in progress

MAX_STEER = 10000

previous_left = None
previous_right = None

TURN_SPEED = 15000

Min_Front_Dist = 50

Circumference = ( math.pi * 40 )

COUNTS_PER_REV = 60

COUNTS_PER_MM = ( COUNTS_PER_REV / Circumference )

MIN_FRONT_DIST = 50
MIN_INCREASE = 30
WALL_THRESHOLD = 60

# ===== MOTOR =====

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
    
# ===== ENCODER =====

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
    
# ===== TURN =====

def Turn_Left_90():
    global Left_Encoder_Count, Right_Encoder_Count

    TURN_MM = (math.pi * 100.0) / 4.0
    TARGET_COUNTS = int(TURN_MM * COUNTS_PER_MM)

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
    time.sleep(0.1)

def Turn_Right_90():
    global Left_Encoder_Count, Right_Encoder_Count
    TURN_MM = (math.pi * 100) / 4.0
    TARGET_COUNTS = int(TURN_MM * COUNTS_PER_MM)

    start_L = Left_Encoder_Count
    start_R = Right_Encoder_Count

    while True:
        dL = abs(Left_Encoder_Count - start_L)
        dR = abs(Right_Encoder_Count - start_R)

        if dL >= TARGET_COUNTS or dR >= TARGET_COUNTS:
            break

        Left_Motor_Set(TURN_SPEED)
        Right_Motor_Set(-TURN_SPEED)

    Motor_Stop()
    time.sleep(0.1)

# ===== PID =====

class PID:
    def __init__(self, Kp, Ki, Kd, integral_limit=None):
        self.Kp = Kp; self.Ki = Ki; self.Kd = Kd
        self.integral = 0.0
        self.last_error = 0.0
        self.integral_limit = integral_limit
        
    def compute(self, target, measurement, dt):
        if dt <= 0: dt = 1e-6
        error = target - measurement
        self.integral += error * dt
        
        if self.integral_limit:
            self.integral = _clamp(self.integral, -self.integral_limit, self.integral_limit)
        
        derivative = (error - self.last_error) / dt
        self.last_error = error
        
        return int(self.Kp*error + self.Ki*self.integral + self.Kd*derivative)

Left_PID = PID(Kp, Ki, Kd, integral_limit=10000)
Right_PID = PID(Kp, Ki, Kd, integral_limit=10000)

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
    
# ===== PID Drive =====

last_L = last_R = 0
last_time = time.ticks_ms()

def PID_Drive(left_mm, right_mm):
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

def Drive_Forward_mm(dist_mm, front_safety_mm=30):
    global Left_Encoder_Count, Right_Encoder_Count
    
    start_L = Left_Encoder_Count
    start_R = Right_Encoder_Count
    Target_Counts = dist_mm * COUNTS_PER_MM
    
    last_count_L = Left_Encoder_Count
    last_count_R = Right_Encoder_Count
    last_time = time.ticks_ms()
    
    while True:
        dL_total = abs(Left_Encoder_Count - start_L)
        dR_total = abs(Right_Encoder_Count - start_R)
        avg_counts = (dL_total + dR_total) / 2.0
        if avg_counts >= Target_Counts:
            break
        
        front_mm = Read_Front_mm()
        if 0 < front_mm <= front_safety_mm:
            break
        
        now = time.ticks_ms()
        dt = time.ticks_diff(now, last_time) / 1000.0
        if dt <= 0:
            dt = 1e-6
            
        if dt < PID_INTERVAL:
            time.sleep(0.002)
            continue
        
        delta_L = Left_Encoder_Count - last_count_L
        delta_R = Right_Encoder_Count - last_count_R

        last_count_L = Left_Encoder_Count
        last_count_R = Right_Encoder_Count
        last_time = now
        
        measured_rps_L = (delta_L / COUNTS_PER_REV) / dt
        measured_rps_R = (delta_R / COUNTS_PER_REV) / dt
        
        Left_PWM = Left_PID.compute(TARGET_RPS, measured_rps_L, dt)
        Right_PWM = Right_PID.compute(TARGET_RPS, measured_rps_R, dt)
        
        Left_PWM = _clamp(Left_PWM, 0, MAX_PWM)
        Right_PWM = _clamp(Right_PWM, 0, MAX_PWM)
        
        Left_Motor_Set(Left_PWM)
        Right_Motor_Set(Right_PWM)
        
        time.sleep(0.005)
        
    Motor_Stop()
    time.sleep(0.05)

def Wall_Turn_Right():
    
    Drive_Forward_mm(40)
    time.sleep(0.5)
    
    Motor_Stop()
    time.sleep(0.5)
    
    Turn_Right_90()
    time.sleep(0.5)
    
    Motor_Stop()
    time.sleep(0.5)
    
    Drive_Forward_mm(90)
    time.sleep(0.5)
    
    Motor_Stop()
    time.sleep(0.5)

def Wall_Turn_Left():
    
    Drive_Forward_mm(40)
    time.sleep(0.5)
    
    Motor_Stop()
    time.sleep(0.5)
    
    Turn_Left_90()
    time.sleep(0.5)
    
    Motor_Stop()
    time.sleep(0.5)
    
    Drive_Forward_mm(90)
    time.sleep(0.5)
    
    Motor_Stop()
    time.sleep(0.5)

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

    RIGHT_OPEN = right_mm > WALL_THRESHOLD
    LEFT_OPEN  = left_mm  > WALL_THRESHOLD

    if RIGHT_OPEN:
        print(" Turning RIGHT ")
        Turn_Right_90()
        time.sleep(0.5)

    elif LEFT_OPEN:
        print(" Turning LEFT ")
        Turn_Left_90()
        time.sleep(0.5)

    else:
        print(" Dead end, Left Turn 90")
        Turn_Left_90()
        time.sleep(0.5)
    return

def detect_corner_or_wall_change(current, previous):
    if previous is None:
        return False, current

    if current <= 0 or current > 200:
        return False, current
    
    delta = current - previous

    corner = delta >= MIN_INCREASE
    wall_appear = delta <= -MIN_INCREASE

    triggered = corner or wall_appear

    return triggered, current

# ===== Special Right-Turn Sequence (side opening) =====

def Execute_Right_Turn_Sequence(avg_right):

    global right_turn_active, right_confirm

    print("Right-turn sequence starting, avg_right =", avg_right)

    right_turn_active = True

    # Step 1: drive forward 60 mm with centering using avg_right
    Drive_Forward_mm_centered(60, avg_right)

    # Step 2: 90° right turn on the spot, centering disabled
    Right_Turn_90()

    # Step 3: drive forward until right ToF matches condition
    while True:
        front_mm = Read_Front_mm()
        if 0 < front_mm <= Min_Front_Dist:
            print("Right-turn sequence: front blocked early")
            Motor_Stop()
            break

        left_raw = Read_Left_mm()
        right_raw = Read_Right_mm()

        # Left centering: use raw unless we're in left opening handling
        left_for_center = left_raw
        if left_miss_active and len(Left_TOF_History) > 0:
            left_for_center = sum(Left_TOF_History) / len(Left_TOF_History)

        # Use avg_right as right centering target
        PID_Drive_Step(left_for_center, avg_right)

        # Condition: equal to avg, less than avg, or within +25% of avg
        if right_raw > 0 and right_raw <= 1.25 * avg_right:
            print("Right-turn sequence: right ToF condition met, value =", right_raw)
            break

        time.sleep(0.01)

    Motor_Stop()
    right_turn_active = False
    right_confirm += 1
    print("Right-turn sequence complete, right_confirm =", right_confirm)


    
try:
    while True:
        front_mm = Read_Front_mm()
        left_mm_raw = Read_Left_mm()
        right_mm_raw = Read_Right_mm()

        if 0 < front_mm <= Min_Front_Dist:
            Motor_Stop()
            Handle_Front_Wall(front_mm)
            continue

        # - Required for detecting left and right turns (use raw ToF) -
        right_triggered, previous_right = detect_corner_or_wall_change(right_mm_raw, previous_right)
        left_triggered, previous_left = detect_corner_or_wall_change(left_mm_raw, previous_left)

        # ===== RIGHT-TURN SIDE OPENING LOGIC =====
        # Detect a right "turn" (opening) while path ahead is clear:
        if (not right_turn_active and
            right_triggered and
            right_mm_raw > Wall_Threshold and
            front_mm > Min_Front_Dist and
            len(Right_TOF_History) == 4):

            avg_right = sum(Right_TOF_History) / len(Right_TOF_History)
            print("Right opening detected, using avg_right =", avg_right)

            # Do NOT modify Right_TOF_History until the entire sequence is done.
            Execute_Right_Turn_Sequence(avg_right)
            continue

        # ===== ToF history (AFTER detection checks) =====

        # --- Forward ToF history: keep most recent 4, delete oldest first ---
        if front_mm > 0:
            Forward_TOF_History.append(front_mm)
            if len(Forward_TOF_History) > 4:
                Forward_TOF_History.pop(0)  # remove oldest

        # --- Right ToF: record only when NOT in right-turn sequence ---
        if right_mm_raw > 0 and not right_turn_active:
            Right_TOF_History.append(right_mm_raw)
            if len(Right_TOF_History) > 4:
                Right_TOF_History.pop(0)  # remove oldest

        # --- Left ToF: history + "opening" handling ---
        # Default: raw left is used for centering
        left_for_center = left_mm_raw

        if left_mm_raw > 0:
            # Fill history up to 4 samples (only when no left-trigger event)
            if len(Left_TOF_History) < 4:
                if not left_triggered:
                    Left_TOF_History.append(left_mm_raw)
                    if len(Left_TOF_History) > 4:
                        Left_TOF_History.pop(0)  # oldest
                # centering stays raw here

            else:
                # We have 4 stored values: compute their average
                avg_left = sum(Left_TOF_History) / len(Left_TOF_History)

                if left_miss_active:
                    # We are currently passing an opening on the left:
                    # use average for centering
                    left_for_center = avg_left

                    # Check if new reading is now within ±25% of avg
                    if avg_left > 0:
                        diff = abs(left_mm_raw - avg_left)
                        if diff <= 0.25 * avg_left and not left_triggered:
                            # Accept this reading, update history (oldest out)
                            Left_TOF_History.pop(0)
                            Left_TOF_History.append(left_mm_raw)

                            # Exit "opening" mode and return to raw centering
                            left_miss_active = False
                            Left_Missed_Count += 1
                            # left_for_center will be raw on next loop

                else:
                    # Not currently in an opening
                    # If a left opening is just detected (big change, side open),
                    # enter "opening" mode and use average for centering
                    if left_triggered and left_mm_raw > Wall_Threshold:
                        left_miss_active = True
                        left_for_center = avg_left
                    else:
                        # No opening: maintain history and use raw for centering
                        Left_TOF_History.pop(0)
                        Left_TOF_History.append(left_mm_raw)
                        # left_for_center remains raw

        else:
            # Invalid left reading; if we have history and we're in opening mode,
            # fall back to average for centering, otherwise leave as-is
            if left_miss_active and len(Left_TOF_History) > 0:
                left_for_center = sum(Left_TOF_History) / len(Left_TOF_History)

        # ===== End ToF history / averaging logic =====

        # Normal straight driving + centering:
        # - left uses left_for_center (raw except during left opening)
        # - right uses raw right_mm_raw (unless in special right-turn sequence)
        PID_Drive_Step(left_for_center, right_mm_raw)
        time.sleep(0.005)
        



# ===== End =====





