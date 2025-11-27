last_left_mm = None
LEFT_WALL_THRESHOLD = 60  # may need to adjust

def ignore_left_openings(left_mm):
    global last_left_mm
    # for when its first run
    if last_left_mm is None:
        last_left_mm = left_mm
    # if there is a wall store that value 
    if 0 < left_mm < LEFT_WALL_THRESHOLD:
        last_left_mm = left_mm
        return left_mm
    # if there is an opening use stored value 
    return last_left_mm

# call function in main loop with: left_mm = ignore_left_openings(left_mm)
# I put it below the left an right triggered bit

def read_sensors():
    left = 40
    right = 80
    front = 120
    #replace with real sensor readings
    return left, right, front

def set_motor_speeds(left_speed, right_speed):
    #replace with real motor code
    pass

Kp = 1.0
setpoint_mm = 40 #change to deired distance from wall?

def pid_control(error):
    return Kp * error

def main():
    driving_out = True
    running = True
    
    while running:
        left_mm, right_mm, front_mm = read_sensors() #read sensors
        left_filtered = ignore_left_openings(left_mm) #use Annabel's function
        
        error = 0
        
        if driving_out:
            if front_mm < 80: #change value accordingly, obstacle detected - end of corridor
                set_motor_speeds(-50, 50)
                driving_out = False
                continue
            
            # PID comtrol for left wall:
            errors = setpoint_mm - left_filtered
            correction = pid_control(error)
            left_speed = 50 + correction
            right_speed = 50 - correction
            set_motor_speeds(left_speed, right_speed)
            
        else:
            error = setpoint_mm - right_mm
            correction = pid_control(error)
            left_speed = 50 - correction
            right_speed = 50 + correction
            set_motor_speeds(left_speed, right_speed)
            
main()
        