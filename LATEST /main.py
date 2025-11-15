from machine import I2C, SoftI2C, Pin, PWM
import time

from vl53l0x import VL53L0X
from vl6180x import VL6180X

# ----- SETTINGS -----
# - PWM -
MAX_PWM = 65535
MAX_DRIVE_PWM = 30000
MIN_PWM = 12500
TARGET_RPS = 15
PID_INTERVAL = 0.05

# - CENTERING - 
CENTER_GAIN = 2
STEER_MAX = 5000

# - TURNING - 
TURN_TIME_90 = 0.25
TURN_SPEED = 15000


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
        DIR1.value(0)
        PWM1.duty_u16(_clamp(speed, 0, MAX_PWM))
    else:
        DIR1.value(1)
        PWM1.duty_u16(_clamp(-speed, 0, MAX_PWM))

# - RIGHT MOTOR CONTROL -
def RMotor_Set(speed):
    speed = int(speed)
    speed = _clamp(speed, -MAX_PWM, MAX_PWM)
    if speed >= 0:
        DIR2.value(0)
        PWM2.duty_u16(_clamp(speed, 0, MAX_PWM))
    else:
        DIR2.value(1)
        PWM2.duty_u16(_clamp(-speed, 0, MAX_PWM))


# ----- TURN SETUP ------
# - LEFT 90 - 
def Turn_Left_90(speed=TURN_SPEED, duration=TURN_TIME_90):
    LMotor_Set(speed)
    RMotor_Set(-speed)
    time.sleep(duration)
    LMotor_Set(0)
    RMotor_Set(0)

# - RIGHT 90 -
def Turn_Right_90(speed=TURN_SPEED, duration=TURN_TIME_90):
    LMotor_Set(-speed)
    RMotor_Set(speed)
    time.sleep(duration)
    LMotor_Set(0)
    RMotor_Set(0)


# ----- ENCODER SETUP -----
# - LEFT MOTOR ENCODER PINS - 
LEncA = Pin(4, Pin.IN, Pin.PULL_UP)
LEncB = Pin(5, Pin.IN, Pin.PULL_UP)

# - RIGHT MOTOR ENCODER PINS - 
REncA = Pin(6, Pin.IN, Pin.PULL_UP)
REncB = Pin(7, Pin.IN, Pin.PULL_UP)

# - SET LEFT ENCODER COUNT = 0 -
LEnc_Count = 0
LPrevA = LEncA.value()

# - SET RIGHT ENCODER COUNT = 0 -
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
# - PID CONTROL -
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
            self.integral = _clamp(self.integral, -self.integral_limit, self.integral_limit)
        derivative = (error - self.last_error) / dt
        self.last_error = error
        return int(self.Kp * error + self.Ki * self.integral + self.Kd * derivative)

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

# - TOF SETUP -
if tof_forward:
    try: tof_forward.start_continuous()
    except: pass

if tof_left:
    try: tof_left.start_range_continuous(50)
    except: pass

if tof_right:
    try: tof_right.start_range_continuous(50)
    except: pass

# - IR SETUP - 
left_ir = Pin(28, Pin.IN)
right_ir = Pin(16, Pin.IN)


# ----- MAIN -----
# - RESET STORED VALUES -
last_L = last_R = 0
last_time = time.ticks_ms()

print("Starting main loop...")

# - RUN LOOP -
try:
    while True:

        # - READ ALL SENSORS -
        front_mm = tof_forward.read_range() if tof_forward else -1
        left_mm  = tof_left.range  if tof_left else -1
        right_mm = tof_right.range if tof_right else -1

        left_ir_val  = left_ir.value()
        right_ir_val = right_ir.value()

        # - FRONT WALL STOP -
        if 0 < front_mm < 75:
            LMotor_Set(0); RMotor_Set(0)
            left_pid.integral = right_pid.integral = 0
            left_pid.last_error = right_pid.last_error = 0
            print("STOP front:", front_mm)
            time.sleep(0.05)
            continue

        # - IR WALL TURNS -
        if left_ir_val == 1:
            print("LEFT IR → LEFT 90°")
            Turn_Left_90()
            continue

        if right_ir_val == 1:
            print("RIGHT IR → RIGHT 90°")
            Turn_Right_90()
            continue
    
        # - MOTOR PID AND CENTERING -
        now = time.ticks_ms()
        dt = time.ticks_diff(now, last_time) / 500.0

        if dt >= PID_INTERVAL:
            last_time = now

            Lcounts = LEnc_Count - last_L
            Rcounts = REnc_Count - last_R
            last_L = LEnc_Count
            last_R = REnc_Count

            L_pwm = left_pid.compute(TARGET_RPS, Lcounts, dt)
            R_pwm = right_pid.compute(TARGET_RPS, Rcounts, dt)

            # - WALL CENTERING -
            if left_mm > 25 and right_mm > 25:
                center_error = right_mm - left_mm
            else:
                center_error = 5

            steering = int(_clamp(center_error * CENTER_GAIN, -STEER_MAX, STEER_MAX))

            L_out = -(L_pwm - steering) 
            R_out = -(R_pwm + steering) 

            # - SPEED LIMITS -
            L_out = _clamp(L_out, -MAX_DRIVE_PWM, MAX_DRIVE_PWM)
            R_out = _clamp(R_out, -MAX_DRIVE_PWM, MAX_DRIVE_PWM)

            LMotor_Set(L_out)
            RMotor_Set(R_out)

        time.sleep(0.1)

except KeyboardInterrupt:
    print("Stopped.")

finally:
    LMotor_Set(0)
    RMotor_Set(0)
    print("Shutdown complete.")


# - END -
