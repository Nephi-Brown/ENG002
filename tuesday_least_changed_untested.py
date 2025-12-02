from machine import I2C, SoftI2C, Pin, PWM
import time
from vl53l0x import VL53L0X
from vl6180x import VL6180X

#sensor initalisation
i2c_front = I2C(1, scl=Pin(19), sda=Pin(18), freq=400000)
i2c_right = SoftI2C(scl=Pin(27), sda=Pin(26), freq=400000)

#scanning for the sensors not hardwiring them
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

#assiging names to the sensors
tof_forward = setup_vl53l0x_on_bus(i2c_front, "front")
tof_right = setup_vl6180x_on_bus(i2c_right, "right")

#sensor objects
if tof_forward:
    try: tof_forward.start_continuous()
    except: pass
if tof_right:
    try: tof_right.start_range_continuous(50)
    except: pass

#functions to read the front and right sensors
def read_front_mm():
    if not tof_forward: return -1
    try: return tof_forward.read_range()
    except: return -1

def read_right_mm():
    if not tof_right: return -1
    try: return tof_right.range
    except: return -1

encoder_l_a = Pin(4, Pin.IN, Pin.PULL_UP) # added pull up 
encoder_l_b = Pin(5, Pin.IN, Pin.PULL_UP)
encoder_r_a = Pin(6, Pin.IN, Pin.PULL_UP)
encoder_r_b = Pin(7, Pin.IN, Pin.PULL_UP)

encoder_countl = 0
encoder_countr = 0

def encoder_call_left(pin):
    global encoder_countl
    if encoder_l_b.value() == 0:
        encoder_countl -= 1
    else:
        encoder_countl += 1 # switched the order of the +/- here as it prevents the robot from turning correctly

def encoder_call_right(pin):
    global encoder_countr
    if encoder_r_b.value() == 0:
        encoder_countr += 1
    else:
        encoder_countr -= 1

encoder_l_a.irq(trigger=Pin.IRQ_RISING, handler=encoder_call_left)
encoder_r_a.irq(trigger=Pin.IRQ_RISING, handler=encoder_call_right)

dir1 = Pin(0, Pin.OUT)  # left motor
dir2 = Pin(2, Pin.OUT)  # right motor
pwm1 = PWM(Pin(1))
pwm2 = PWM(Pin(3))
pwm1.freq(20000) # increased the freq of motors to stop the high pitched sound when they're on 
pwm2.freq(20000)

def correct_motors(correction_fix):
    normal_speed = 50
    fix_left = normal_speed - correction_fix
    fix_right = normal_speed + correction_fix
    if fix_left > 100: fix_left = 100
    if fix_left < -100: fix_left = -100
    if fix_right > 100: fix_right = 100
    if fix_right < -100: fix_right = -100
    motor_pwm(fix_left, pwm1, dir1)
    motor_pwm(fix_right, pwm2, dir2)

def correct_motors_custom(fix_left, fix_right):
    if fix_left > 100: fix_left = 100
    if fix_left < -100: fix_left = -100
    if fix_right > 100: fix_right = 100
    if fix_right < -100: fix_right = -100
    motor_pwm(fix_left, pwm1, dir1)
    motor_pwm(fix_right, pwm2, dir2)

def motor_pwm(speed, pwm_object, dir_pin):
    speed = speed * 0.3
    if pwm_object == pwm2:  # RIGHT motor flip
        if speed >= 0:
            dir_pin.value(1) # changed to 1 to stop spinning 
            pwm_object.duty_u16(int(speed * 655))
        else:
            dir_pin.value(0) # changed to 0 to stop spinning
            pwm_object.duty_u16(int(-speed * 655))
    else:  # LEFT motor normal
        if speed >= 0:
            dir_pin.value(1)
            pwm_object.duty_u16(int(speed * 655))
        else:
            dir_pin.value(0)
            pwm_object.duty_u16(int(-speed * 655))

def stop():
    pwm1.duty_u16(0)
    pwm2.duty_u16(0)

def wait(ms):
    time.sleep_ms(ms)

def turn_left(count_target, base_speed=50):
    global encoder_countl, encoder_countr
    encoder_countl = 0
    encoder_countr = 0
    while encoder_countr < count_target:
        correct_motors_custom(-base_speed, base_speed)
        time.sleep_ms(1)
    stop()

def turn_right(count_target, base_speed=50):
    global encoder_countl, encoder_countr
    encoder_countl = 0
    encoder_countr = 0
    while encoder_countl < count_target:
        correct_motors_custom(base_speed, -base_speed)
        time.sleep_ms(1)
    stop()

def right_wall_following(right_mm, front_mm):
    global pid_active, encoder_countr, encoder_countl # added encoder counts to globals 
    if right_mm > gap:
        pid_active = False
        stop()
        encoder_countr = 0
        encoder_countl = 0
        while encoder_countr < gap_roll_counts and encoder_countl < gap_roll_counts:
            correct_motors_custom(50, 50)
            time.sleep_ms(1)
        stop()
        turn_right(counts)
        wait(50)
        pid_active = True

    elif front_mm < safe_front:
        pid_active = False
        while front_mm < safe_front:
            front_mm = tof_forward.read_range()
            right_mm = tof_right.range
            print("During left turn - Front:", front_mm, "Right:", right_mm)  # debug both sensors
            turn_left(counts)
            wait(50)
        stop()
        pid_active = True    

    elif right_mm < gap and front_mm > safe_front:
        pid_active = True
        pass
counts = 15
gap_roll_counts = 20
gap = 150
safe_front = 60
pid_active = True

P = 400
I = 25
D = 1
centered_distance = 60
previous_error = 0
error_added = 0

try:
    while True:
        front_mm = tof_forward.read_range() if tof_forward else -1
        right_mm = tof_right.range if tof_right else -1

        # main debug line for both sensors
        print("Main loop - Front:", front_mm, "Right:", right_mm)

        if pid_active and right_mm != -1:
            error = centered_distance - right_mm
            error_added += error
            error_change = error - previous_error
            correction_fix = (P * error) + (I * error_added) + (D * error_change)
        else:
            correction_fix = 0
            error = 0

        correct_motors(correction_fix)
        right_wall_following(right_mm, front_mm) # might need to change the order of this function 
        previous_error = error
        time.sleep(0.1)

except KeyboardInterrupt:
    pass
finally:
    if tof_forward:
        try:
            tof_forward.stop_continuous()
        except:
            pass
    if tof_right:
        try:
            tof_right.stop_range_continuous()
        except:
            pass
    stop()