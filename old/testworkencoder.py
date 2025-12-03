from machine import Pin
import time

# Motor control pins (GP2, GP3)
in1 = Pin(2, Pin.OUT)   # GP2
in2 = Pin(3, Pin.OUT)   # GP3

# Encoder pins (Channel A: GP5, Channel B: GP6)
encoder_a = Pin(17, Pin.IN, Pin.PULL_UP)
encoder_b = Pin(16, Pin.IN, Pin.PULL_UP)

# Movement counter (positive or negative)
position = 0

# Distance per pulse (you can change this)
DISTANCE_PER_PULSE = 1.0  # e.g., mm or degrees

# Interrupt handler for encoder
def encoder_callback(pin):
    global position
    if encoder_b.value() == 1:
        position += 1  # Forward
    else:
        position -= 1  # Backward

# Attach interrupt to Channel A (rising edge)
encoder_a.irq(trigger=Pin.IRQ_RISING, handler=encoder_callback)

print("Motor and encoder direction tracking starting...")

try:
    while True:
        position = 0

        # Spin forward
        in1.value(1)
        in2.value(0.00025)
        print("Motor spinning forward...")
        time.sleep(2)
        in1.value(0)
        in2.value(0)
        time.sleep(0.2)

        distance_moved = position * DISTANCE_PER_PULSE
        print("Moved FORWARD by", abs(distance_moved), "units")

        time.sleep(2)

        # Spin backward
        position = 0
        in1.value(0)
        in2.value(1)
        print("Motor spinning backward...")
        time.sleep(2)
        in1.value(0)
        in2.value(0)
        time.sleep(0.2)

        distance_moved = position * DISTANCE_PER_PULSE
        print("Moved BACKWARD by", abs(distance_moved), "units")

        time.sleep(2)

except KeyboardInterrupt:
    in1.value(0)
    in2.value(0)
    print("Test ended, motor stopped.")
