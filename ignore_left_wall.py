last_left_mm = None
LEFT_WALL_THRESHOLD = 60  # may need to adjust

def ignore_left_openings(left_mm):
    global last_left_mm, LEFT_WALL_THRESHOLD
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
