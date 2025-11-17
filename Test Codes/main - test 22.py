from machine import I2C, SoftI2C, Pin, PWM
import time

from vl53l0x import VL53L0X
from vl6180x import VL6180X

# ----- SETTINGS -----
# - PWM -
MAX_PWM       = 15000
MAX_DRIVE_PWM = 65535
MIN_PWM       = 12500
TARGET_RPS    = 10
PID_INTERVAL  = 0.05  # s

# - CENTERING -
CENTER_GAIN = 150
STEER_MAX   = 5000

# - TURNING -
TURN_TIME_90 = 0.4   # s, tune for your robot
TURN_SPEED   = 10000

# - DISTANCE / ENCODER CALIBRATION -
COUNTS_PER_MM = 2.0   # encoder counts per mm (TUNE THIS!)

# - FRONT WALL THRESHOLDS (mm) -
FRONT_TURN_DIST   = 55    # 7cm -> initial left 90
FRONT_REPEAT_DIST = 80   # 10cm -> repeat 90 if still too close

# - RIGHT-OPENING BEHAVIOUR (mm) -
IR_OPEN_FORWARD_MM      = 100    # 8cm forward before turning right
IR_OPEN_EXTRA_FORWARD_MM = 50   # 5cm extra when side distance halves
IR_OPEN_FORWARD_SHORT_MM = 125 

SIDE_DROP_FACTOR = 0.5   # "less than half recorded value"

# - FINISH DETECTION -
FINISH_TOLERANCE = 0.10  # 10%


# ----- MOTOR SETUP -----
# - LEFT MOTOR -
DIR1 = Pin(0, Pin.OUT)
PWM1 = PWM(Pin(1)); PWM1.freq(20000)

# - RIGHT MOTOR -
DIR2 = Pin(2, Pin.OUT)
PWM2 = PWM(Pin(3)); PWM2.freq(20000)


def _clamp(val, lo, hi):
    return lo if val < lo else (hi if val > hi else val)


# - LEFT MOTOR CONTROL -
def LMotor_Set(speed):
    speed = int(speed)
    speed = _clamp(speed, -MAX_PWM, MAX_PWM)
    if speed >= 0:
        DIR1.value(1)
        PWM1.duty_u16(_clamp(speed, 0, MAX_PWM))
    else:
        DIR1.value(0)
        PWM1.duty_u16(_clamp(-speed, 0, MAX_PWM))


# - RIGHT MOTOR CONTROL -
def RMotor_Set(speed):
    speed = int(speed)
    speed = _clamp(speed, -MAX_PWM, MAX_PWM)
    if speed >= 0:
        DIR2.value(1)
        PWM2.duty_u16(_clamp(speed, 0, MAX_PWM))
    else:
        DIR2.value(0)
        PWM2.duty_u16(_clamp(-speed, 0, MAX_PWM))


def Stop_Motors():
    LMotor_Set(0)
    RMotor_Set(0)


# ----- TURN SETUP ------
#  (wheels spin in opposite directions only)

def Turn_Left_90(speed=TURN_SPEED, duration=TURN_TIME_90):
    LMotor_Set(speed)
    RMotor_Set(-speed)
    time.sleep(duration)
    Stop_Motors()


def Turn_Right_90(speed=TURN_SPEED, duration=TURN_TIME_90):
    LMotor_Set(-speed)
    RMotor_Set(speed)
    time.sleep(duration)
    Stop_Motors()


# ----- ENCODER SETUP -----
# - LEFT MOTOR ENCODER PINS -
LEncA = Pin(4, Pin.IN, Pin.PULL_UP)
LEncB = Pin(5, Pin.IN, Pin.PULL_UP)

# - RIGHT MOTOR ENCODER PINS -
REncA = Pin(6, Pin.IN, Pin.PULL_UP)
REncB = Pin(7, Pin.IN, Pin.PULL_UP)

# - SET ENCODER COUNTS = 0 -
LEnc_Count = 0
LPrevA = LEncA.value()

REnc_Count = 0
RPrevA = REncA.value()


# - LEFT ENCODER CONTROL -
def Left_Encoder(pin):
    global LEnc_Count, LPrevA
    A = LEncA.value(); B = LEncB.value()
    if A != LPrevA:
        LEnc_Count += 1 if A == B else -1
        LPrevA = A


# - RIGHT ENCODER CONTROL -
def Right_Encoder(pin):
    global REnc_Count, RPrevA
    A = REncA.value(); B = REncB.value()
    if A != RPrevA:
        REnc_Count += -1 if A == B else 1
        RPrevA = A


# - INCREASE / DECREASE ENCODER VALUES -
LEncA.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=Left_Encoder)
REncA.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=Right_Encoder)


# ----- PID SETUP -----
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


# - SET PID FOR LEFT AND RIGHT -
left_pid  = PID(2000, 50, 0, integral_limit=10000)
right_pid = PID(2000, 50, 0, integral_limit=10000)


# ----- TOF SENSOR SETUP -----
# - FRONT TOF SETUP -
def setup_vl53l0x_on_bus(i2c, label):
    for addr in i2c.scan():
        try:
            s = VL53L0X(i2c, address=addr)
            print(label, "VL53L0X at", hex(addr))
            return s
        except:
            pass
    return None


# - LEFT / RIGHT TOF SETUP -
def setup_vl6180x_on_bus(i2c, label):
    for addr in i2c.scan():
        try:
            s = VL6180X(i2c, address=addr)
            print(label, "VL6180X at", hex(addr))
            return s
        except:
            pass
    return None


# ----- I2C / SOFT I2C SETUP -----
# - FIND TOF SENSORS -
i2c_forward = I2C(1, sda=Pin(18), scl=Pin(19), freq=400000)
i2c_left    = SoftI2C(sda=Pin(20), scl=Pin(21), freq=400000)
i2c_right   = SoftI2C(sda=Pin(26), scl=Pin(27), freq=400000)

# - SET SENSOR NAMES -
tof_forward = setup_vl53l0x_on_bus(i2c_forward, "forward")
tof_left    = setup_vl6180x_on_bus(i2c_left,  "left")
tof_right   = setup_vl6180x_on_bus(i2c_right, "right")

# - TOF START -
if tof_forward:
    try: tof_forward.start_continuous()
    except: pass
if tof_left:
    try: tof_left.start_range_continuous(50)
    except: pass
if tof_right:
    try: tof_right.start_range_continuous(50)
    except: pass

# ---------- SENSOR HELPERS ----------

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


# ---------- PID DRIVE STEP (CENTERING FROM PID Straight ToF and Enc.py) ----------

last_L = last_R = 0
last_time = time.ticks_ms()


def PID_Drive_Step(left_mm, right_mm):
    """One PID + centering update, now using centering logic from PID Straight ToF and Enc.py."""
    global last_L, last_R, last_time

    now = time.ticks_ms()
    dt = time.ticks_diff(now, last_time) / 500.0

    if dt < PID_INTERVAL:
        return  # wait until next period

    last_time = now

    # --- encoder counts this interval ---
    Lcounts = LEnc_Count - last_L
    Rcounts = REnc_Count - last_R
    last_L = LEnc_Count
    last_R = REnc_Count

    # --- base PID output (speed control only) ---
    L_pwm = left_pid.compute(TARGET_RPS, Lcounts, dt)
    R_pwm = right_pid.compute(TARGET_RPS, Rcounts, dt)

    # Apply raw PID (so wheels keep moving even if no centering)
    LMotor_Set(L_pwm)
    RMotor_Set(R_pwm)

    # ============================================================
    # CENTERING SECTION (copied from PID Straight ToF and Enc.py)
    # ============================================================
    if left_mm > 0 and right_mm > 0:
        # Want left_mm ≈ right_mm
        side_error = left_mm - right_mm      # want this = 0
        Kp_center = 80                       # steering gain — TUNE THIS

        steering = Kp_center * side_error    # positive → steer right

        # Saturate steering so motors don't reverse unexpectedly
        if steering > 15000:
            steering = 15000
        if steering < -15000:
            steering = -15000

        # Apply differential correction ON TOP of PID speed control
        L_corrected = L_pwm - steering
        R_corrected = R_pwm + steering

        # Clamp to safe range
        L_corrected = _clamp(L_corrected, -MAX_PWM, MAX_PWM)
        R_corrected = _clamp(R_corrected, -MAX_PWM, MAX_PWM)

        # Send corrected speeds to motors
        LMotor_Set(L_corrected)
        RMotor_Set(R_corrected)

# - Turn Forward - 

def PID_Drive_Step_Turns(left_mm, right_mm):
    """One PID + centering update, now using centering logic from PID Straight ToF and Enc.py."""
    global last_L, last_R, last_time

    now = time.ticks_ms()
    dt = time.ticks_diff(now, last_time) / 500.0

    if dt < PID_INTERVAL:
        return  # wait until next period

    last_time = now

    # --- encoder counts this interval ---
    Lcounts = LEnc_Count - last_L
    Rcounts = REnc_Count - last_R
    last_L = LEnc_Count
    last_R = REnc_Count

    # --- base PID output (speed control only) ---
    L_pwm = left_pid.compute(TARGET_RPS, Lcounts, dt)
    R_pwm = right_pid.compute(TARGET_RPS, Rcounts, dt)

    # Apply raw PID (so wheels keep moving even if no centering)
    LMotor_Set(L_pwm)
    RMotor_Set(R_pwm)


# ---------- DISTANCE DRIVE HELPERS ----------

def Drive_Forward_mm(dist_mm, front_safety_mm=40):
    """
    Drive forward approximately dist_mm using encoders,
    while PID + centering drive the motors.
    """
    global LEnc_Count, REnc_Count

    start_L = LEnc_Count
    start_R = REnc_Count
    target_counts = dist_mm * COUNTS_PER_MM

    while True:
        # Distance moved so far
        dL = abs(LEnc_Count - start_L)
        dR = abs(REnc_Count - start_R)
        avg_counts = (dL + dR) / 2.0
        if avg_counts >= target_counts:
            break

        front_mm = Read_Front_mm()
        left_mm  = Read_Left_mm()
        right_mm = Read_Right_mm()

        # Safety: if something is very close in front, stop
        if 0 < front_mm <= front_safety_mm:
            break

        PID_Drive_Step(left_mm, right_mm)
        time.sleep(0.01)

    Stop_Motors()


def Drive_Until_Front(target_mm):
    """Drive forward until front ToF is <= target_mm."""
    while True:
        front_mm = Read_Front_mm()
        left_mm  = Read_Left_mm()
        right_mm = Read_Right_mm()

        if front_mm > 0 and front_mm <= target_mm:
            break

        PID_Drive_Step(left_mm, right_mm)
        time.sleep(0.01)

    Stop_Motors()


# ---------- FINISH-LINE TRACKING ----------

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
        return abs(x - avg) <= FINISH_TOLERANCE * avg

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
    Turn_Left_90()
    time.sleep(0.1)

    # 3) New front distance
    d2 = Read_Front_mm()
    if d2 <= 0:
        d2 = 100
    half2 = d2 / 2.0

    # 4) Drive half of that
    print("Finish: drive", half2, "mm forward")
    Drive_Forward_mm(half2)

    # 5) Spin left 3 times on the spot
    print("Finish: spin left 3x")
    for _ in range(12):  # 12 x 90° = 3 full rotations
        Turn_Left_90()
        time.sleep(0.05)

    Stop_Motors()
    print("Maze solved! Holding position.")
    while True:
        Stop_Motors()
        time.sleep(1)


# ---------- FRONT-WALL HANDLING ----------

def Handle_Front_Wall(front_mm):
    """
    When front ToF <= 7cm:
      - turn 90° left on the spot
      - measure front distance
      - record it
      - if still <= 10cm, repeat 90° left
    After the sequence, check for finish condition.
    """
    print("Front wall at", front_mm, "mm -> left 90 sequence")
    left_pid.integral = right_pid.integral = 0
    left_pid.last_error = right_pid.last_error = 0

    while True:
        Turn_Left_90()
        time.sleep(0.1)

        new_front = Read_Front_mm()
        print("Front after turn:", new_front, "mm")
        Record_Front_Turn_Distance(new_front)

        if 0 < new_front <= FRONT_REPEAT_DIST:
            # Another 90° left is required
            continue
        else:
            break

    if Finish_Condition():
        print("Finish condition detected!")
        Finish_Sequence()


# - turn right -

Right_Count = 0

def Turn_Right_Motion():
    global Right_Count  # Declare Right_Count as global

    print("forward 10cm, right 90")

    Right_Count += 1

    Stop_Motors()
    time.sleep(1.0)

    # record encoder before move
    start_R = REnc_Count

    # if robot already moved (safety)
    if (REnc_Count - start_R) > 5:
        Stop_Motors()
        return

    # move forward first
    while True:
        front_mm = Read_Front_mm()
        left_mm  = Read_Left_mm()
        right_mm = Read_Right_mm()

        if front_mm > 0 and front_mm < 50 :
            break

        PID_Drive_Step_Turns(left_mm, right_mm)
        time.sleep(0.01)

    Stop_Motors()

    time.sleep(1.0)

    # turn
    Turn_Right_90()
    time.sleep(1.0)

    # short forward after turn
    Drive_Forward_mm(IR_OPEN_FORWARD_SHORT_MM)
    time.sleep(1.0)


# - turn left -

Left_Count = 0

def Turn_Left_Motion():
    global Left_Count  # Declare Left_Count as global

    print("forward 10cm, left 90")

    Left_Count += 1  # Corrected this line

    Stop_Motors()
    time.sleep(1.0)

    # record encoder before move
    start_L = LEnc_Count

    # check for movement already
    if (LEnc_Count - start_L) > 5:
        Stop_Motors()
        return

    # move forward first
    while True:
        front_mm = Read_Front_mm()
        left_mm  = Read_Left_mm()
        right_mm = Read_Right_mm()

        if front_mm > 0 and front_mm < 50 :
            break

        PID_Drive_Step_Turns(left_mm, right_mm)
        time.sleep(0.01)

    Stop_Motors()

    time.sleep(1.0)

    # turn
    Turn_Left_90()
    time.sleep(1.0)

    # forward after turn
    Drive_Forward_mm(IR_OPEN_FORWARD_SHORT_MM)
    time.sleep(1.0)


# ----- MAIN -----

print("Starting maze solver (right-wall follow)...")

try:
    while True:
        front_mm = Read_Front_mm()
        left_mm  = Read_Left_mm()
        right_mm = Read_Right_mm()

        # 1) FRONT WALL HANDLING
        if 0 < front_mm <= FRONT_TURN_DIST:
            Stop_Motors()
            Handle_Front_Wall(front_mm)
            continue

        if (left_mm - 25) > 100:
            Turn_Left_Motion()
            print("Left_Count")

        if (right_mm - 25) > 100:
            Turn_Right_Motion()
            print("Right_Count")

        # 3) NORMAL FORWARD DRIVE WITH PID + CENTERING
        PID_Drive_Step(left_mm, right_mm)
        time.sleep(0.01)

except KeyboardInterrupt:
    print("Stopped by user.")

finally:
    #Stop_Motors()
    print("Shutdown complete.")
