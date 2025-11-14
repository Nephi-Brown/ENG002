# main.py
# Read all ToF + IR sensors on a Pico W and print CSV lines.
#
# Pins (must stay as given):
#   Forward ToF  (VL53L0X "53") : SDA GP18, SCL GP19
#   Left ToF     (VL6180X "61") : SDA GP20, SCL GP21
#   Right ToF    (VL6180X "61") : SDA GP26, SCL GP27
#   Left IR                          GP28 (digital)
#   Right IR                         GP16 (digital)
#
# Required files in the filesystem:
#   - vl53l0x.py
#   - vl6180x.py

from machine import I2C, SoftI2C, Pin
import time

from vl53l0x import VL53L0X
from vl6180x import VL6180X


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

# ---------------- MAIN LOOP ----------------

SAMPLE_HZ = 20
DELAY = 1 / SAMPLE_HZ

print("front_mm,left_mm,right_mm,leftIR,rightIR")
time.sleep(1.0)

try:
    while True:
        # Forward distance (VL53L0X)
        if tof_forward:
            try:
                front_mm = tof_forward.read_range()
            except Exception:
                front_mm = -1
        else:
            front_mm = -1

        # Left distance (VL6180X)
        if tof_left:
            try:
                left_mm = tof_left.range
            except Exception:
                left_mm = -1
        else:
            left_mm = -1

        # Right distance (VL6180X)
        if tof_right:
            try:
                right_mm = tof_right.range
            except Exception:
                right_mm = -1
        else:
            right_mm = -1

        # IRs
        left_ir_val = left_ir.value()
        right_ir_val = right_ir.value()

        # One CSV line per sample
        print("{},{},{},{},{}".format(
            front_mm, left_mm, right_mm, left_ir_val, right_ir_val
        ))

        time.sleep(DELAY)

except KeyboardInterrupt:
    pass
finally:
    # Clean shutdown of continuous modes
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
