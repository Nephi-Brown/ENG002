# main.py
# Find a VL6180X at ANY I2C address, change it to NEW_ADDR, then stream mm readings.
# Requires vl6180x_mpy.py on the device.

from machine import I2C, Pin
import time
from vl6180x import VL6180X

# ------------- CONFIG -------------
I2C_ID = 0          # Pico I2C0: GP0=SDA, GP1=SCL ; I2C1: GP2=SDA, GP3=SCL
SDA_PIN = 8
SCL_PIN = 9
I2C_FREQ = 400_000

# The new I2C address you want (7-bit, typical usable range 0x08..0x77)
NEW_ADDR = 0x2A

# Optional XSHUT hardware reset (set True and wire XSHUT to XSHUT_PIN)
USE_XSHUT = False
XSHUT_PIN = 2

# Streaming config
CONT_PERIOD_MS = 120
PRINT_PERIOD_MS = 120
# ----------------------------------

EXPECTED_MODEL_ID = 0xB4            # VL6180X model ID
REG_MODEL_ID = 0x000
REG_ADDR = 0x0212                   # I2C_SLAVE__DEVICE_ADDRESS

def make_i2c():
    return I2C(I2C_ID, sda=Pin(SDA_PIN), scl=Pin(SCL_PIN), freq=I2C_FREQ)

def pulse_xshut():
    if not USE_XSHUT:
        return
    x = Pin(XSHUT_PIN, Pin.OUT, value=0)
    time.sleep_ms(5)
    x.value(1)
    time.sleep_ms(5)

def scan_hex(i2c):
    return [hex(a) for a in i2c.scan()]

def read_model_id(i2c, addr):
    """Try to read 8-bit model ID from 16-bit register 0x000. Returns int or None."""
    try:
        data = i2c.readfrom_mem(addr, REG_MODEL_ID, 1, addrsize=16)
        return data[0]
    except OSError:
        return None

def find_all_vl6180x(i2c):
    """Return a list of addresses on the bus that positively identify as VL6180X."""
    found = []
    for addr in i2c.scan():
        mid = read_model_id(i2c, addr)
        if mid == EXPECTED_MODEL_ID:
            found.append(addr)
    return found

def change_address(i2c, current_addr, new_addr):
    """Write new_addr to the device at current_addr; verify and return new address."""
    # Safety checks
    if not (0x08 <= new_addr <= 0x77):
        raise ValueError("NEW_ADDR must be 0x08..0x77 (7-bit).")
    # Write new address to I2C_SLAVE__DEVICE_ADDRESS (takes effect immediately)
    try:
        i2c.writeto_mem(current_addr, REG_ADDR, bytes([new_addr & 0x7F]), addrsize=16)
    except OSError:
        raise RuntimeError("Failed writing new address to device at 0x%02X." % current_addr)
    time.sleep_ms(2)
    # Verify by probing model ID at the new address
    mid = read_model_id(i2c, new_addr)
    if mid != EXPECTED_MODEL_ID:
        raise RuntimeError("Device did not respond at new address 0x%02X." % new_addr)
    return new_addr

def main():
    i2c = make_i2c()
    print("I2C devices at start:", scan_hex(i2c))

    # Optional clean hardware reset
    pulse_xshut()

    # Find all VL6180X devices regardless of current address
    addrs = find_all_vl6180x(i2c)
    if len(addrs) == 0:
        # Try once more after XSHUT pulse (if not used initially)
        if not USE_XSHUT:
            pulse_xshut()
            time.sleep_ms(10)
            addrs = find_all_vl6180x(i2c)
    if len(addrs) == 0:
        raise RuntimeError("No VL6180X found on the bus. Check wiring/power.")

    if len(addrs) > 1:
        # To safely handle multiple parts, you'd normally hold others in XSHUT low.
        raise RuntimeError(
            "Multiple VL6180X detected at %s. "
            "Use XSHUT pins to bring them up one-by-one before readdressing."
            % ", ".join([hex(a) for a in addrs])
        )

    current = addrs[0]
    print("Found VL6180X at", hex(current))

    if current == NEW_ADDR:
        print("Already at desired address", hex(NEW_ADDR))
    else:
        # Ensure NEW_ADDR not already in use by some other device
        in_use = i2c.scan()
        if NEW_ADDR in in_use:
            raise RuntimeError(
                "NEW_ADDR %s already in use on the bus. Choose another." % hex(NEW_ADDR)
            )
        newa = change_address(i2c, current, NEW_ADDR)
        print("Changed address:", hex(current), "→", hex(newa))

    print("I2C devices now:", scan_hex(i2c))

    # Create driver at the NEW_ADDR and stream distances
    sensor = VL6180X(i2c, address=NEW_ADDR)
    sensor.start_range_continuous(period=CONT_PERIOD_MS)

    print("Streaming distance (mm) from", hex(NEW_ADDR), "— Ctrl+C to stop.")
    try:
        while True:
            d = sensor.range
            if d is None:
                d = 0
            elif d > 255:
                d = 255
            print(d)  # numeric-first for Thonny Plotter
            time.sleep_ms(PRINT_PERIOD_MS)
    except KeyboardInterrupt:
        pass
    finally:
        sensor.stop_range_continuous()
        print("Stopped.")

if __name__ == "__main__":
    main()