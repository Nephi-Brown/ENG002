# main.py — Simple VL53L0X serial plotter output (mm per line)

from machine import I2C, Pin
import time
from vl53l0x import VL53L0X

# Pico/Pico W default I2C0: SDA=GP4, SCL=GP5 (adjust if you wired differently)
i2c = I2C(1, sda=Pin(6), scl=Pin(7), freq=400_000)

# Create sensor (0x29 default). Add io_timeout_s if you want (e.g., 0.5)
tof = VL53L0X(i2c, address=0x29)

# Optional: continuous mode for faster reads
tof.start_continuous()

SAMPLE_HZ = 10          # plot at ~10 samples per second
DELAY = 1 / SAMPLE_HZ

print("VL53L0X distance (mm) — one value per line for serial plotters")
time.sleep(1.0)

try:
    while True:
        # For continuous mode we can call read_range() directly
        d_mm = tof.read_range()
        # Print just the number so plotters auto-detect it
        print(d_mm)
        time.sleep(DELAY)

except KeyboardInterrupt:
    pass
finally:
    tof.stop_continuous()
    print("Stopped.")
