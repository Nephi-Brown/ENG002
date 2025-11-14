from machine import Pin, PWM, I2C
import time

# ==========================
# Motor Setup
# ==========================

# Left Motor
DIR1 = Pin(0, Pin.OUT)
PWM1 = PWM(Pin(1))
PWM1.freq(20000)

def LMotor_Set(speed):
    # speed = -65535 .. +65535
    if speed >= 0:
        DIR1.value(1)
        PWM1.duty_u16(speed)
    else:
        DIR1.value(0)
        PWM1.duty_u16(-speed)

# Right Motor
DIR2 = Pin(2, Pin.OUT)
PWM2 = PWM(Pin(3))
PWM2.freq(20000)

def RMotor_Set(speed):
    if speed >= 0:
        DIR2.value(1)
        PWM2.duty_u16(speed)
    else:
        DIR2.value(0)
        PWM2.duty_u16(-speed)

# ==========================
# Speed Limits
# ==========================

MAX_PWM = 65535
MIN_PWM = 10000      # prevents stall
TARGET_RPS = 20     # example speed target (20 encoder counts per PID interval)

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
        REnc_Count += -1 if A == B else 1
        RPrevA = A


# Attach interrupts
LEncA.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=Left_Encoder)
REncA.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=Right_Encoder)


# ==========================
# PID Controller
# ==========================

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

# ==========================
# Main Loop — PID Speed Control
# ==========================

print("Starting PID drive...")

last_L = 0
last_R = 0
last_time = time.ticks_ms()

while True:
    now = time.ticks_ms()
    dt = time.ticks_diff(now, last_time) / 1000  # seconds

    if dt >= 0.05:  # 50ms PID update rate
        last_time = now

        # --- compute speed from encoder counts ---
        Lcounts = LEnc_Count - last_L
        Rcounts = REnc_Count - last_R

        last_L = LEnc_Count
        last_R = REnc_Count

        # measured speed = encoder ticks per PID interval
        L_speed = Lcounts
        R_speed = Rcounts

        # --- run PID ---
        L_pwm = left_pid.compute(TARGET_RPS, L_speed, dt)
        R_pwm = right_pid.compute(TARGET_RPS, R_speed, dt)

        # --- apply motor power ---
        LMotor_Set(L_pwm)
        RMotor_Set(R_pwm)

        print("L:", L_speed, "R:", R_speed, "PWM:", L_pwm, R_pwm)

