from machine import I2C, SoftI2C, Pin, PWM, I2C
import time

from vl53l0x import VL53L0X
from vl6180x import VL6180X

# ===========
# Motor Setup
# ===========

# Left Motor
DIR1 = Pin(0, Pin.OUT)
PWM1 = PWM(Pin(1))
PWM1.freq(20000)

def LMotor_Set(speed):
    speed = int(speed)
    speed = -speed
    if speed >= 0:
        DIR1.value(0)
        PWM1.duty_u16(speed)
    else:
        DIR1.value(1)
        PWM1.duty_u16(-speed)

# Right Motor
DIR2 = Pin(2, Pin.OUT)
PWM2 = PWM(Pin(3))
PWM2.freq(20000)

def RMotor_Set(speed):
    speed = int(speed)
    speed = -speed
    if speed >= 0:
        DIR2.value(0)
        PWM2.duty_u16(speed)
    else:
        DIR2.value(1)
        PWM2.duty_u16(-speed)

# ============
# Speed Limits
# ============

MAX_PWM = 65535
MIN_PWM = 20000      # prevents stall
TARGET_RPS = 25     # example speed target (20 encoder counts per PID interval)

# ==============
# Motor Encoders
# ==============

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
        REnc_Count += -1 if A == B else 1
        RPrevA = A


# Attach interrupts
LEncA.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=Left_Encoder)
REncA.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=Right_Encoder)


# ==============
# PID Controller
# ==============

class PID:
    def __init__(self, Kp, Ki, Kd, output_min=-65535, output_max=65535):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.min = output_min
        self.max = output_max

        self.integral = 0
        self.last_error = 0

    def compute(self, target, measurement, dt):
        error = target - measurement

        # P
        P = self.Kp * error

        # I (with anti-windup)
        self.integral += error * dt
        self.integral = max(min(self.integral, self.max), self.min)
        I = self.Ki * self.integral

        # D
        derivative = (error - self.last_error) / dt
        D = self.Kd * derivative

        self.last_error = error

        output = P + I + D

        # Clamp
        if output > self.max:
            output = self.max
        elif output < self.min:
            output = self.min

        return int(output)

# PID parameters — tune these
left_pid = PID(Kp=2000, Ki=50, Kd=0)
right_pid = PID(Kp=2000, Ki=50, Kd=0)

# =============================
# Main Loop — PID Speed Control
# =============================

print("Starting PID drive...")

last_L = 0
last_R = 0
last_time = time.ticks_ms()





# =============
# Nephi Sensors
# =============

# ---------- Helpers to auto-detect sensors on a bus ----------

def setup_vl53l0x_on_bus(i2c, label="forward"):
    """Scan i2c bus, try each address as VL53L0X, return (sensor, addr) or (None, None)."""
    addrs = i2c.scan()
    if not addrs:
        print("[{}] No I2C devices found on this bus.".format(label))
        return None, None

    for addr in addrs:
        try:
            s = VL53L0X(i2c, address=addr)
            print("[{}] VL53L0X found at 0x{:02X}".format(label, addr))
            return s, addr
        except Exception:
            # Not our sensor, keep trying others on the bus
            pass

    print("[{}] Could not init VL53L0X on any address: {}".format(
        label, [hex(a) for a in addrs]
    ))
    return None, None


def setup_vl6180x_on_bus(i2c, label="side"):
    """Scan i2c bus, try each address as VL6180X, return (sensor, addr) or (None, None)."""
    addrs = i2c.scan()
    if not addrs:
        print("[{}] No I2C devices found on this bus.".format(label))
        return None, None

    for addr in addrs:
        try:
            s = VL6180X(i2c, address=addr)
            print("[{}] VL6180X found at 0x{:02X}".format(label, addr))
            return s, addr
        except OSError:
            # Wrong device/address, keep searching
            pass
        except Exception as e:
            print("[{}] Error trying addr 0x{:02X}: {}".format(label, addr, e))

    print("[{}] Could not init VL6180X on any address: {}".format(
        label, [hex(a) for a in addrs]
    ))
    return None, None


# ---------------- I2C BUS SETUP ----------------

# Forward VL53L0X on hardware I2C1 (GP18/GP19)
i2c_forward = I2C(1, sda=Pin(18), scl=Pin(19), freq=400_000)

# Left VL6180X on its own software I2C bus (GP20/GP21)
i2c_left = SoftI2C(sda=Pin(20), scl=Pin(21), freq=400_000)

# Right VL6180X on its own software I2C bus (GP26/GP27)
i2c_right = SoftI2C(sda=Pin(26), scl=Pin(27), freq=400_000)

# ---------------- CREATE SENSOR OBJECTS ----------------

# Forward VL53L0X
tof_forward, addr_forward = setup_vl53l0x_on_bus(i2c_forward, "forward")
if tof_forward:
    tof_forward.start_continuous()

# Left VL6180X
tof_left, addr_left = setup_vl6180x_on_bus(i2c_left, "left")
if tof_left:
    tof_left.start_range_continuous(period=50)

# Right VL6180X
tof_right, addr_right = setup_vl6180x_on_bus(i2c_right, "right")
if tof_right:
    tof_right.start_range_continuous(period=50)

# ---------------- IR SENSORS ----------------

# Assuming digital outputs (0/1)
left_ir = Pin(28, Pin.IN)
right_ir = Pin(16, Pin.IN)

def setup_vl53l0x_on_bus(i2c, label="forward"):
    """Scan i2c bus, try each address as VL53L0X, return (sensor, addr) or (None, None)."""
    addrs = i2c.scan()
    if not addrs:
        print("[{}] No I2C devices found on this bus.".format(label))
        return None, None

    for addr in addrs:
        try:
            s = VL53L0X(i2c, address=addr)
            print("[{}] VL53L0X found at 0x{:02X}".format(label, addr))
            return s, addr
        except Exception:
            # Not our sensor, keep trying others on the bus
            pass

    print("[{}] Could not init VL53L0X on any address: {}".format(
        label, [hex(a) for a in addrs]
    ))
    return None, None


def setup_vl6180x_on_bus(i2c, label="side"):
    """Scan i2c bus, try each address as VL6180X, return (sensor, addr) or (None, None)."""
    addrs = i2c.scan()
    if not addrs:
        print("[{}] No I2C devices found on this bus.".format(label))
        return None, None

    for addr in addrs:
        try:
            s = VL6180X(i2c, address=addr)
            print("[{}] VL6180X found at 0x{:02X}".format(label, addr))
            return s, addr
        except OSError:
            # Wrong device/address, keep searching
            pass
        except Exception as e:
            print("[{}] Error trying addr 0x{:02X}: {}".format(label, addr, e))

    print("[{}] Could not init VL6180X on any address: {}".format(
        label, [hex(a) for a in addrs]
    ))
    return None, None


# ---------------- I2C BUS SETUP ----------------

# Forward VL53L0X on hardware I2C1 (GP18/GP19)
i2c_forward = I2C(1, sda=Pin(18), scl=Pin(19), freq=400_000)

# Left VL6180X on its own software I2C bus (GP20/GP21)
i2c_left = SoftI2C(sda=Pin(20), scl=Pin(21), freq=400_000)

# Right VL6180X on its own software I2C bus (GP26/GP27)
i2c_right = SoftI2C(sda=Pin(26), scl=Pin(27), freq=400_000)

# ---------------- CREATE SENSOR OBJECTS ----------------

# Forward VL53L0X
tof_forward, addr_forward = setup_vl53l0x_on_bus(i2c_forward, "forward")
if tof_forward:
    tof_forward.start_continuous()

# Left VL6180X
tof_left, addr_left = setup_vl6180x_on_bus(i2c_left, "left")
if tof_left:
    tof_left.start_range_continuous(period=50)

# Right VL6180X
tof_right, addr_right = setup_vl6180x_on_bus(i2c_right, "right")
if tof_right:
    tof_right.start_range_continuous(period=50)

# ---------------- IR SENSORS ----------------

# Assuming digital outputs (0/1)
left_ir = Pin(28, Pin.IN)
right_ir = Pin(16, Pin.IN)

# ---------- Helpers to auto-detect sensors on a bus ----------

def setup_vl53l0x_on_bus(i2c, label="forward"):
    """Scan i2c bus, try each address as VL53L0X, return (sensor, addr) or (None, None)."""
    addrs = i2c.scan()
    if not addrs:
        print("[{}] No I2C devices found on this bus.".format(label))
        return None, None

    for addr in addrs:
        try:
            s = VL53L0X(i2c, address=addr)
            print("[{}] VL53L0X found at 0x{:02X}".format(label, addr))
            return s, addr
        except Exception:
            # Not our sensor, keep trying others on the bus
            pass

    print("[{}] Could not init VL53L0X on any address: {}".format(
        label, [hex(a) for a in addrs]
    ))
    return None, None


def setup_vl6180x_on_bus(i2c, label="side"):
    """Scan i2c bus, try each address as VL6180X, return (sensor, addr) or (None, None)."""
    addrs = i2c.scan()
    if not addrs:
        print("[{}] No I2C devices found on this bus.".format(label))
        return None, None

    for addr in addrs:
        try:
            s = VL6180X(i2c, address=addr)
            print("[{}] VL6180X found at 0x{:02X}".format(label, addr))
            return s, addr
        except OSError:
            # Wrong device/address, keep searching
            pass
        except Exception as e:
            print("[{}] Error trying addr 0x{:02X}: {}".format(label, addr, e))

    print("[{}] Could not init VL6180X on any address: {}".format(
        label, [hex(a) for a in addrs]
    ))
    return None, None


# ---------------- I2C BUS SETUP ----------------

# Forward VL53L0X on hardware I2C1 (GP18/GP19)
i2c_forward = I2C(1, sda=Pin(18), scl=Pin(19), freq=400_000)

# Left VL6180X on its own software I2C bus (GP20/GP21)
i2c_left = SoftI2C(sda=Pin(20), scl=Pin(21), freq=400_000)

# Right VL6180X on its own software I2C bus (GP26/GP27)
i2c_right = SoftI2C(sda=Pin(26), scl=Pin(27), freq=400_000)

# ---------------- CREATE SENSOR OBJECTS ----------------

# Forward VL53L0X
tof_forward, addr_forward = setup_vl53l0x_on_bus(i2c_forward, "forward")
if tof_forward:
    tof_forward.start_continuous()

# Left VL6180X
tof_left, addr_left = setup_vl6180x_on_bus(i2c_left, "left")
if tof_left:
    tof_left.start_range_continuous(period=50)

# Right VL6180X
tof_right, addr_right = setup_vl6180x_on_bus(i2c_right, "right")
if tof_right:
    tof_right.start_range_continuous(period=50)

# ---------------- IR SENSORS ----------------

# Assuming digital outputs (0/1)
left_ir = Pin(28, Pin.IN)
right_ir = Pin(16, Pin.IN)

# If left IR is analog instead, replace above with:
# from machine import ADC
# left_ir_adc = ADC(28)
# and in the loop use: left_ir_val = left_ir_adc.read_u16()

SAMPLE_HZ = 20
DELAY = 1 / SAMPLE_HZ

print("front_mm,left_mm,right_mm,leftIR,rightIR")
time.sleep(1.0)


try:
    while True:

        # - Motor Code -
        now = time.ticks_ms()
        dt = time.ticks_diff(now, last_time) / 1000  # seconds

        if dt >= 0.05:  # 50ms PID update rate
            last_time = now

            # --- compute speed from encoder counts ---
            Lcounts = LEnc_Count - last_L
            Rcounts = REnc_Count - last_R

            last_L = LEnc_Count
            last_R = REnc_Count

            L_speed = Lcounts
            R_speed = Rcounts

            # --- run PID ---
            L_pwm = left_pid.compute(TARGET_RPS, L_speed, dt)
            R_pwm = right_pid.compute(TARGET_RPS, R_speed, dt)

            LMotor_Set(L_pwm)
            RMotor_Set(R_pwm)

            print("L:", L_speed, "R:", R_speed, "PWM:", L_pwm, R_pwm)

        # - Sensor Code -

        if tof_forward:
            try:
                front_mm = tof_forward.read_range()
            except Exception:
                front_mm = -1
        else:
            front_mm = -1

        if tof_left:
            try:
                left_mm = tof_left.range
            except Exception:
                left_mm = -1
        else:
            left_mm = -1

        if tof_right:
            try:
                right_mm = tof_right.range
            except Exception:
                right_mm = -1
        else:
            right_mm = -1

        left_ir_val = left_ir.value()
        right_ir_val = right_ir.value()

        print("{},{},{},{},{}".format(
            front_mm, left_mm, right_mm, left_ir_val, right_ir_val
        ))

        # ===============================
        # --- SAFETY STOP (front < 50) ---
        # ===============================
        if front_mm > 0 and front_mm < 50:
            LMotor_Set(0)
            RMotor_Set(0)
            print("STOP — FRONT TOO CLOSE!")
            time.sleep(0.1)
            continue



        # ============================================================
        # --- CENTERING CONTROL (use difference of left/right ToF) ---
        # ============================================================

        # If either side sensor fails, skip centering
        if left_mm > 0 and right_mm > 0:

            side_error = left_mm - right_mm      # want this = 0
            Kp_center = 80                       # steering gain — TUNE THIS

            steering = Kp_center * side_error    # positive → turn right

            # Saturate steering so motors don't reverse unexpectedly
            if steering > 15000:
                steering = 15000
            if steering < -15000:
                steering = -15000

            # Apply differential correction ON TOP of PID speed control
            L_corrected = L_pwm - steering
            R_corrected = R_pwm + steering

            # Clamp to safe range
            L_corrected = max(min(L_corrected, MAX_PWM), -MAX_PWM)
            R_corrected = max(min(R_corrected, MAX_PWM), -MAX_PWM)

            # Send corrected speeds to motors
            LMotor_Set(L_corrected)
            RMotor_Set(R_corrected)

            print("Centering: err=", side_error,
                  " steer=", steering,
                  " Lc=", L_corrected,
                  " Rc=", R_corrected)


        time.sleep(DELAY)

except KeyboardInterrupt:
    pass

finally:
    if tof_forward:
        try:
            tof_forward.stop_continuous()
        except:
            pass

    if tof_left:
        try:
            tof_left.stop_range_continuous()
        except:
            pass

    if tof_right:
        try:
            tof_right.stop_range_continuous()
        except:
            pass

    print("Stopped all sensors.")
