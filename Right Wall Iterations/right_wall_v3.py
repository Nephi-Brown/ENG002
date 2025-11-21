from machine import I2C, SoftI2C, Pin, PWM
import time

from vl53l0x import VL53L0X
from vl6180x import VL6180X

# ----- SETTINGS -----
# - PWM -
MAX_PWM       = 65535
MAX_DRIVE_PWM = 15000
MIN_PWM       = 10000
TARGET_RPS    = 10
PID_INTERVAL  = 0.05  # s

# - CENTERING -
# (Enabled only when LEFT IR = 1)
CENTER_GAIN = 150
STEER_MAX   = 15000

# - TURNING -
TURN_TIME_90 = 0.375
TURN_SPEED   = 10000

# - DISTANCE CALIBRATION -
COUNTS_PER_MM = 2.0

# - FRONT WALL THRESHOLDS -
FRONT_TURN_DIST   = 60
FRONT_REPEAT_DIST = 70

# - RIGHT OPENING BEHAVIOUR -
IR_OPEN_FORWARD_MM       = 80
IR_OPEN_EXTRA_FORWARD_MM = 40
SIDE_DROP_FACTOR = 0.5

# - FINISH DETECTION -
FINISH_TOLERANCE = 0.10

# ----- MOTOR SETUP -----
DIR1 = Pin(0, Pin.OUT)
PWM1 = PWM(Pin(1)); PWM1.freq(20000)

DIR2 = Pin(2, Pin.OUT)
PWM2 = PWM(Pin(3)); PWM2.freq(20000)

def _clamp(val, lo, hi):
    return lo if val < lo else (hi if val > hi else val)

def LMotor_Set(speed):
    speed = _clamp(int(speed), -MAX_PWM, MAX_PWM)
    if speed >= 0:
        DIR1.value(1)
        PWM1.duty_u16(speed)
    else:
        DIR1.value(0)
        PWM1.duty_u16(-speed)

def RMotor_Set(speed):
    speed = _clamp(int(speed), -MAX_PWM, MAX_PWM)
    if speed >= 0:
        DIR2.value(1)
        PWM2.duty_u16(speed)
    else:
        DIR2.value(0)
        PWM2.duty_u16(-speed)

def Stop_Motors():
    LMotor_Set(0)
    RMotor_Set(0)


# ----- TURNING -----
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


# ----- ENCODERS -----
LEncA = Pin(4, Pin.IN, Pin.PULL_UP)
LEncB = Pin(5, Pin.IN, Pin.PULL_UP)
REncA = Pin(6, Pin.IN, Pin.PULL_UP)
REncB = Pin(7, Pin.IN, Pin.PULL_UP)

LEnc_Count = 0; LPrevA = LEncA.value()
REnc_Count = 0; RPrevA = REncA.value()

def Left_Encoder(pin):
    global LEnc_Count, LPrevA
    A = LEncA.value(); B = LEncB.value()
    if A != LPrevA:
        LEnc_Count += 1 if A == B else -1
        LPrevA = A

def Right_Encoder(pin):
    global REnc_Count, RPrevA
    A = REncA.value(); B = REncB.value()
    if A != RPrevA:
        REnc_Count += -1 if A == B else 1
        RPrevA = A

LEncA.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=Left_Encoder)
REncA.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=Right_Encoder)


# ----- PID -----
class PID:
    def __init__(self, Kp, Ki, Kd, limit=None):
        self.Kp = Kp; self.Ki = Ki; self.Kd = Kd
        self.integral = 0; self.last_error = 0
        self.limit = limit

    def compute(self, target, measurement, dt):
        if dt <= 0: dt = 1e-6
        error = target - measurement
        self.integral += error * dt
        if self.limit:
            self.integral = _clamp(self.integral, -self.limit, self.limit)
        d = (error - self.last_error) / dt
        self.last_error = error
        return int(self.Kp*error + self.Ki*self.integral + self.Kd*d)

left_pid  = PID(2000, 50, 0, 10000)
right_pid = PID(2000, 50, 0, 10000)

# ----- CENTERING (imported logic from main) -----
def apply_centering_from_main(L_pwm, R_pwm, left_mm, right_mm, drive_limit=MAX_DRIVE_PWM):
    """
    Ported centering from main: if both side distances are valid,
    compute steering = Kp(=80) * (left_mm - right_mm), clamp to ±15000,
    then subtract/add to left/right PWM respectively, finally clamp to drive_limit.
    """
    if left_mm > 0 and right_mm > 0:
        Kp_Center = 80
        steering = Kp_Center * (left_mm - right_mm)
        steering = _clamp(steering, -15000, 15000)
        Lc = _clamp(L_pwm - steering, -drive_limit, drive_limit)
        Rc = _clamp(R_pwm + steering, -drive_limit, drive_limit)
        return Lc, Rc
    return L_pwm, R_pwm


# ----- SENSOR SETUP -----
def setup_vl53(i2c):
    for a in i2c.scan():
        try: return VL53L0X(i2c, address=a)
        except: pass
    return None

def setup_vl618(i2c):
    for a in i2c.scan():
        try: return VL6180X(i2c, address=a)
        except: pass
    return None

i2c_forward = I2C(1, sda=Pin(18), scl=Pin(19), freq=400000)
i2c_left    = SoftI2C(sda=Pin(20), scl=Pin(21), freq=400000)
i2c_right   = SoftI2C(sda=Pin(26), scl=Pin(27), freq=400000)

tof_forward = setup_vl53(i2c_forward)
tof_left    = setup_vl618(i2c_left)
tof_right   = setup_vl618(i2c_right)

if tof_forward: tof_forward.start_continuous()
if tof_left:    tof_left.start_range_continuous(50)
if tof_right:   tof_right.start_range_continuous(50)


# ----- IR SETUP -----
right_ir = Pin(16, Pin.IN)
left_ir  = Pin(28, Pin.IN)   # NEW: controls centering only


# ----- SENSOR READERS -----
def Read_Front_mm():
    try: return tof_forward.read_range()
    except: return -1

def Read_Left_mm():
    try: return tof_left.range
    except: return -1

def Read_Right_mm():
    try: return tof_right.range
    except: return -1


# ----- PID + CONDITIONAL CENTERING -----
last_L = last_R = 0
last_time = time.ticks_ms()

def PID_Drive_Step(left_mm, right_mm):
    global last_L, last_R, last_time

    now = time.ticks_ms()
    dt = time.ticks_diff(now, last_time) / 500
    if dt < PID_INTERVAL: return
    last_time = now

    Lcounts = LEnc_Count - last_L
    Rcounts = REnc_Count - last_R
    last_L = LEnc_Count
    last_R = REnc_Count

    L_pwm = left_pid.compute(TARGET_RPS, Lcounts, dt)
    R_pwm = right_pid.compute(TARGET_RPS, Rcounts, dt)

    # base PID output
    LMotor_Set(L_pwm)
    RMotor_Set(R_pwm)

    # ============================================================
    # CENTERING ACTIVE ONLY WHEN LEFT IR = 1
    # ============================================================
    if left_ir.value() == 1:
        # Centering from main()
        L_corrected, R_corrected = apply_centering_from_main(L_pwm, R_pwm, left_mm, right_mm, drive_limit=MAX_DRIVE_PWM)
        LMotor_Set(L_corrected)
        RMotor_Set(R_corrected)


# ----- FORWARD MOTION HELPERS -----
def Drive_Forward_mm(dist_mm, front_safety_mm=40):
    global LEnc_Count, REnc_Count
    start_L = LEnc_Count; start_R = REnc_Count
    target = dist_mm * COUNTS_PER_MM

    while True:
        moved = ((abs(LEnc_Count-start_L) + abs(REnc_Count-start_R))/2)
        if moved >= target: break

        front = Read_Front_mm()
        if 0 < front <= front_safety_mm: break

        PID_Drive_Step(Read_Left_mm(), Read_Right_mm())
        time.sleep(0.01)

    Stop_Motors()

def Drive_Until_Front(target_mm):
    while True:
        front = Read_Front_mm()
        if front > 0 and front <= target_mm:
            break
        PID_Drive_Step(Read_Left_mm(), Read_Right_mm())
        time.sleep(0.01)
    Stop_Motors()


# ----- FINISH-LINE TRACKING -----
front_turn_history = []

def Record_Front_Turn_Distance(d):
    if d > 0:
        front_turn_history.append(d)
        if len(front_turn_history) > 3:
            front_turn_history.pop(0)

def Finish_Condition():
    if len(front_turn_history) < 3: return False
    a,b,c = front_turn_history
    avg = (a+b+c)/3
    return (abs(a-avg)<=avg*FINISH_TOLERANCE and
            abs(b-avg)<=avg*FINISH_TOLERANCE and
            abs(c-avg)<=avg*FINISH_TOLERANCE)

def Finish_Sequence():
    last = front_turn_history[-1]
    half = last/2
    Drive_Until_Front(half)
    Turn_Left_90()
    d2 = Read_Front_mm()
    Drive_Forward_mm(d2/2)
    for _ in range(12): Turn_Left_90()
    Stop_Motors()
    while True: pass


# ----- FRONT WALL HANDLING -----
def Handle_Front_Wall(front_mm):
    while True:
        Turn_Left_90()
        new = Read_Front_mm()
        Record_Front_Turn_Distance(new)
        if 0 < new <= FRONT_REPEAT_DIST:
            continue
        break

    if Finish_Condition():
        Finish_Sequence()


# ----- RIGHT OPENING HANDLING -----
def Handle_Right_Open():
    Drive_Forward_mm(IR_OPEN_FORWARD_MM)
    Turn_Right_90()
    baseL = Read_Left_mm(); baseR = Read_Right_mm()

    while True:
        front = Read_Front_mm()
        if 0 < front <= FRONT_TURN_DIST:
            Handle_Front_Wall(front)
            return

        left = Read_Left_mm(); right = Read_Right_mm()

        drop = False
        if baseL>0 and left>0 and left < baseL*SIDE_DROP_FACTOR: drop=True
        if baseR>0 and right>0 and right < baseR*SIDE_DROP_FACTOR: drop=True

        if drop:
            Drive_Forward_mm(IR_OPEN_EXTRA_FORWARD_MM)
            return

        PID_Drive_Step(left, right)
        time.sleep(0.01)


# ----- MAIN LOOP -----
print("Starting maze solver...")

try:
    while True:
        front = Read_Front_mm()
        left  = Read_Left_mm()
        right = Read_Right_mm()
        IRr   = right_ir.value()

        if 0 < front <= FRONT_TURN_DIST:
            Stop_Motors()
            Handle_Front_Wall(front)
            continue

        if IRr == 0:
            Stop_Motors()
            Handle_Right_Open()
            continue

        PID_Drive_Step(left, right)
        time.sleep(0.01)

except KeyboardInterrupt:
    pass

finally:
    Stop_Motors()
