from machine import I2C, SoftI2C, Pin, PWM
import time
from vl53l0x import VL53L0X
from vl6180x import VL6180X

#snesor initalisation
i2c_front = I2C(1, scl=Pin(19), sda=Pin(18), freq=400000)
i2c_right = SoftI2C(scl=Pin(27), sda=Pin(26), freq=400000)
i2c_left = SoftI2C(scl=Pin(21), sda=Pin(20), freq=400000)

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
tof_left = setup_vl6180x_on_bus(i2c_left, "left")
#sensor objects
if tof_forward:
    tof_forward.start_continuous()
if tof_right:
    tof_right.start_range_continuous(period=50)
if tof_left:
    tof_left.start_range_continuous(period=50)
    

encoder_l_a = Pin(4, Pin.IN)
encoder_l_b = Pin(5, Pin.IN)
encoder_r_a = Pin(6, Pin.IN)
encoder_r_b = Pin(7, Pin.IN)

encoder_countl = 0
encoder_countr = 0

def encoder_call_left(pin):
    global encoder_countl
    if encoder_l_b.value() == 0:
        encoder_countl -= 1
    else:
        encoder_countl += 1

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
pwm1.freq(20000)
pwm2.freq(20000)


def motor_pwm(speed, pwm_object, dir_pin, forward_dir=1):
    speed = speed * 0.4
    duty = int(abs(speed) * 655)
    if speed >= 0:
        dir_pin.value(forward_dir)
        pwm_object.duty_u16(duty)
    else:
        dir_pin.value(1 - forward_dir)
        pwm_object.duty_u16(duty)


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

def stop():
    pwm1.duty_u16(0)
    pwm2.duty_u16(0)

def wait(ms):
    time.sleep_ms(ms)

def turn_left(count_target, base_speed=55):
    global encoder_countl, encoder_countr
    encoder_countl = 0
    encoder_countr = 0
    while abs(encoder_countl) < count_target and abs(encoder_countr) < count_target:
        # Left motor backward, right motor forward
        motor_pwm(-base_speed, pwm1, dir1, forward_dir=1)
        motor_pwm(base_speed, pwm2, dir2, forward_dir=1)

        # Keep sensors alive during the turn
        front_mm = tof_forward.read_range() if tof_forward else -1
        right_mm = tof_right.range if tof_right else -1
        print("Turning left - Front:", front_mm, "Right:", right_mm)

        time.sleep_ms(1)
    stop()


def turn_right(count_target, base_speed=55):
    global encoder_countl, encoder_countr
    encoder_countl = 0
    encoder_countr = 0
    while abs(encoder_countl) < count_target and abs(encoder_countr) < count_target:
        # Left motor forward, right motor backward
        motor_pwm(base_speed, pwm1, dir1, forward_dir=1)
        motor_pwm(-base_speed, pwm2, dir2, forward_dir=1)

        # Keep sensors alive during the turn
        front_mm = tof_forward.read_range() if tof_forward else -1
        right_mm = tof_right.range if tof_right else -1
        print("Turning right - Front:", front_mm, "Right:", right_mm)

        time.sleep_ms(1)
    stop()


   

def right_wall_following(right_mm, front_mm):
    global pid_active
    global encoder_countr
    global encoder_countl
    global mode

    if mode == "forward":
        # add hysteresis margins so noise doesn't retrigger
        if right_mm > gap + 20:   # margin above gap
            mode = "turning_right"
            pid_active = False
            stop()
            wait(1000)
            if front_mm > safe_front - 10:
                encoder_countr = 0
                encoder_countl = 0
                while encoder_countr < gap_roll_counts and encoder_countl < gap_roll_counts:
                    correct_motors_custom(48, 50)
                    time.sleep_ms(1)
                stop()
                wait(1000)
                turn_right(countsr)
                wait(1000)
                encoder_countr = 0
                encoder_countl = 0
                while encoder_countr < gap_roll_countss and encoder_countl < gap_roll_countss: #this may thro logic 
                    correct_motors_custom(50, 50)
                    time.sleep_ms(1)
                stop() 
            pid_active = True
            mode = "forward"
        elif right_mm > gap +20 and left_mm > gap + 20:
            mode = "turning right again"
            pid_active = False
            turn_left(countsl)
            stop()
            wait(1000)
            while encoder_countr < gap_roll_countst and encoder_countl < gap_roll_countst: #this may thro logic 
                correct_motors_custom(50, 50)
                time.sleep_ms(1)
            stop()
            pid_active = True
            mode = "forward"
        elif front_mm < safe_front - 10:  # margin below safe_front
            mode = "turning_left"
            pid_active = False
            stop()
            wait(1000)
            while front_mm < safe_front - 10:
                front_mm = tof_forward.read_range()
                right_mm = tof_right.range
                print("During left turn - Front:", front_mm, "Right:", right_mm)
                turn_left(countsl)
                stop()
                wait(50)
            stop()
            pid_active = True
            mode = "forward"

        elif right_mm < gap and front_mm > safe_front:
            pid_active = True
            
            pass
        
countsr = 30
countsl = 15
gap_roll_counts = 50
gap_roll_countss = 40
gap_roll_countst = 45
gap = 50
safe_front = 60
pid_active = True


P = 0.97
I = 0.1
D = 1.79
centered_distance = 30
previous_error = 0
error_added = 0
error_addedl = 0
previous_errorl = 0
mode = "forward"
try:
    while True:
        front_mm = tof_forward.read_range() if tof_forward else -1
        right_mm = tof_right.range if tof_right else -1
        left_mm = tof_left.range if tof_left else -1 
        
        right_wall_following(right_mm, front_mm)#calling the function first

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
    if tof_left:
        try:
            tof_left.stop_range_continuous()
        except:
            pass
    stop()
