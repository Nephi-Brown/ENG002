"""
vl6180x_mpy.py — MicroPython driver for VL6180X (ToF + ALS)

Drop-in replacement adapted from Adafruit's CircuitPython driver,
but using machine.I2C directly (no adafruit_bus_device needed).

Tested on Raspberry Pi Pico (rp2) MicroPython with I2C addrsize=16 support.

Typical usage:
    from machine import I2C, Pin
    from vl6180x_mpy import VL6180X

    i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400_000)
    tof = VL6180X(i2c)  # default address 0x29
    print(tof.range)    # mm (0..255). 255 often means "invalid/out of range".
"""

import time
from micropython import const

__all__ = ["VL6180X", "ALS_GAIN_1", "ALS_GAIN_1_25", "ALS_GAIN_1_67", "ALS_GAIN_2_5",
           "ALS_GAIN_5", "ALS_GAIN_10", "ALS_GAIN_20", "ALS_GAIN_40"]

# --- Registers (16-bit addresses) ---
_VL6180X_REG_IDENTIFICATION_MODEL_ID = const(0x000)

_VL6180X_REG_SYSTEM_HISTORY_CTRL = const(0x012)
_VL6180X_REG_SYSTEM_INTERRUPT_CONFIG = const(0x014)
_VL6180X_REG_SYSTEM_INTERRUPT_CLEAR = const(0x015)
_VL6180X_REG_SYSTEM_FRESH_OUT_OF_RESET = const(0x016)

_VL6180X_REG_SYSRANGE_START = const(0x018)
_VL6180X_REG_SYSRANGE_INTERMEASUREMENT_PERIOD = const(0x01B)
_VL6180X_REG_SYSRANGE_PART_TO_PART_RANGE_OFFSET = const(0x024)

_VL6180X_REG_SYSALS_START = const(0x038)
_VL6180X_REG_SYSALS_ANALOGUE_GAIN = const(0x03F)
_VL6180X_REG_SYSALS_INTEGRATION_PERIOD_HI = const(0x040)
_VL6180X_REG_SYSALS_INTEGRATION_PERIOD_LO = const(0x041)

_VL6180X_REG_RESULT_RANGE_STATUS = const(0x04D)
_VL6180X_REG_RESULT_INTERRUPT_STATUS_GPIO = const(0x04F)
_VL6180X_REG_RESULT_ALS_VAL = const(0x050)
_VL6180X_REG_RESULT_HISTORY_BUFFER_0 = const(0x052)
_VL6180X_REG_RESULT_RANGE_VAL = const(0x062)

# Defaults/constants
_VL6180X_DEFAULT_I2C_ADDR = const(0x29)
_EXPECTED_MODEL_ID = const(0xB4)  # VL6180X

# User-facing ALS gains
ALS_GAIN_1 = const(0x06)
ALS_GAIN_1_25 = const(0x05)
ALS_GAIN_1_67 = const(0x04)
ALS_GAIN_2_5 = const(0x03)
ALS_GAIN_5 = const(0x02)
ALS_GAIN_10 = const(0x01)
ALS_GAIN_20 = const(0x00)
ALS_GAIN_40 = const(0x07)

# Error/status codes (upper nibble of RESULT_RANGE_STATUS)
ERROR_NONE = const(0)
ERROR_SYSERR_1 = const(1)
ERROR_SYSERR_5 = const(5)
ERROR_ECEFAIL = const(6)
ERROR_NOCONVERGE = const(7)
ERROR_RANGEIGNORE = const(8)
ERROR_SNR = const(11)
ERROR_RAWUFLOW = const(12)
ERROR_RAWOFLOW = const(13)
ERROR_RANGEUFLOW = const(14)
ERROR_RANGEOFLOW = const(15)


class VL6180X:
    """
    MicroPython driver for VL6180X.

    :param i2c: machine.I2C instance
    :param address: I2C address (default 0x29)
    :param offset: signed mm offset applied via PART_TO_PART_RANGE_OFFSET
    :param strict_id: if True, raise if model ID != 0xB4 (VL6180X)
    """

    def __init__(self, i2c, address=_VL6180X_DEFAULT_I2C_ADDR, offset=0, strict_id=True):
        self._i2c = i2c
        self._addr = address

        mid = self._read8(_VL6180X_REG_IDENTIFICATION_MODEL_ID)
        if strict_id and mid != _EXPECTED_MODEL_ID:
            raise RuntimeError(
                "Unexpected model ID 0x%02X (expected 0x%02X). Is this a VL6180X?" % (mid, _EXPECTED_MODEL_ID)
            )
        if not strict_id and mid != _EXPECTED_MODEL_ID:
            print("Warning: model ID 0x%02X != VL6180X (0x%02X). Proceeding anyway." % (mid, _EXPECTED_MODEL_ID))

        self._load_settings()
        self._write8(_VL6180X_REG_SYSTEM_FRESH_OUT_OF_RESET, 0x00)
        self.offset = offset  # property setter writes register

        # If device was left in continuous mode by a prior session, stop it
        if self.continuous_mode_enabled:
            self.stop_range_continuous()
            time.sleep_ms(100)

        # Enable history buffer for RANGE by default
        self._write8(_VL6180X_REG_SYSTEM_HISTORY_CTRL, 0x01)

    # -------- Public API --------

    @property
    def range(self):
        """Return distance in mm (0..255). 255 typically means invalid/out-of-range."""
        if self.continuous_mode_enabled:
            return self._read_range_continuous()
        return self._read_range_single()

    def start_range_continuous(self, period=100):
        """
        Start continuous ranging.

        :param period: time between measurements in ms. Valid 20..2550 in 10ms steps.
        """
        if not 20 <= period <= 2550:
            raise ValueError("period must be 20..2550 ms")
        # Register expects units of 10ms, value is (period/10)-1
        period_reg = (period // 10) - 1
        self._write8(_VL6180X_REG_SYSRANGE_INTERMEASUREMENT_PERIOD, period_reg)
        self._write8(_VL6180X_REG_SYSRANGE_START, 0x03)  # continuous

    def stop_range_continuous(self):
        """Stop continuous ranging."""
        val = self._read8(_VL6180X_REG_SYSRANGE_START)
        if (val & 0x02) != 0:
            # Clear continuous bit; datasheet suggests writing 0x01 to leave in single-shot armed state
            self._write8(_VL6180X_REG_SYSRANGE_START, 0x01)

    @property
    def continuous_mode_enabled(self):
        """True if continuous ranging bit is set."""
        val = self._read8(_VL6180X_REG_SYSRANGE_START)
        # Bit1 indicates continuous operation when set alongside internal state
        return (val & 0x02) != 0

    @property
    def offset(self):
        """Signed mm offset applied at PART_TO_PART_RANGE_OFFSET."""
        return self._offset

    @offset.setter
    def offset(self, off):
        # Register is signed 8-bit
        if off < -128 or off > 127:
            raise ValueError("offset must fit in int8 (-128..127)")
        if off < 0:
            off = 256 + off
        self._write8(_VL6180X_REG_SYSRANGE_PART_TO_PART_RANGE_OFFSET, off & 0xFF)
        self._offset = off if off < 128 else off - 256

    # ---- ALS (lux) ----
    def read_lux(self, gain):
        """Read ambient light in lux. Choose one of ALS_GAIN_* constants."""
        reg = self._read8(_VL6180X_REG_SYSTEM_INTERRUPT_CONFIG)
        reg &= ~0x38
        reg |= 0x4 << 3  # IRQ on ALS ready
        self._write8(_VL6180X_REG_SYSTEM_INTERRUPT_CONFIG, reg)

        # 100 ms integration
        self._write8(_VL6180X_REG_SYSALS_INTEGRATION_PERIOD_HI, 0)
        self._write8(_VL6180X_REG_SYSALS_INTEGRATION_PERIOD_LO, 100)

        gain = min(gain, ALS_GAIN_40)
        self._write8(_VL6180X_REG_SYSALS_ANALOGUE_GAIN, 0x40 | gain)

        # Start ALS
        self._write8(_VL6180X_REG_SYSALS_START, 0x01)

        # Wait until "New Sample Ready threshold event"
        while ((self._read8(_VL6180X_REG_RESULT_INTERRUPT_STATUS_GPIO) >> 3) & 0x7) != 4:
            pass

        lux = self._read16(_VL6180X_REG_RESULT_ALS_VAL)

        # Clear interrupt
        self._write8(_VL6180X_REG_SYSTEM_INTERRUPT_CLEAR, 0x07)

        # Convert to lux (datasheet scale ~0.32 count/lux then divide by gain)
        lux *= 0.32
        if gain == ALS_GAIN_1:
            pass
        elif gain == ALS_GAIN_1_25:
            lux /= 1.25
        elif gain == ALS_GAIN_1_67:
            lux /= 1.67
        elif gain == ALS_GAIN_2_5:
            lux /= 2.5
        elif gain == ALS_GAIN_5:
            lux /= 5
        elif gain == ALS_GAIN_10:
            lux /= 10
        elif gain == ALS_GAIN_20:
            lux /= 20
        elif gain == ALS_GAIN_40:
            lux /= 40

        # Integration time correction (100 ms -> *100/100 = 1.0 here)
        lux *= 100
        lux /= 100
        return lux

    @property
    def range_status(self):
        """Upper nibble of RESULT_RANGE_STATUS (0==OK)."""
        return self._read8(_VL6180X_REG_RESULT_RANGE_STATUS) >> 4

    @property
    def range_from_history(self):
        """Latest range from history buffer (if enabled), else None."""
        if not self.range_history_enabled:
            return None
        return self._read8(_VL6180X_REG_RESULT_HISTORY_BUFFER_0)

    @property
    def ranges_from_history(self):
        """Last 16 ranges from history buffer (if enabled), else None."""
        if not self.range_history_enabled:
            return None
        return [self._read8(_VL6180X_REG_RESULT_HISTORY_BUFFER_0 + i) for i in range(16)]

    @property
    def range_history_enabled(self):
        """True if history buffer stores RANGE values."""
        hist = self._read8(_VL6180X_REG_SYSTEM_HISTORY_CTRL)
        # 0x01 means "store RANGE". 0x00 disabled. other values store ALS.
        return (hist & 0x01) == 0x01

    # -------- Internals --------

    def _read_range_single(self):
        """Single-shot read using the same handshake as Adafruit's driver."""
        # Wait until "range ready" bit (bit0) is set
        while (self._read8(_VL6180X_REG_RESULT_RANGE_STATUS) & 0x01) == 0:
            pass
        # Start single range
        self._write8(_VL6180X_REG_SYSRANGE_START, 0x01)
        # Then read via the same continuous read path (handles interrupt clear)
        return self._read_range_continuous()

    def _read_range_continuous(self):
        """Read one sample in continuous mode, waiting for GPIO int."""
        # Wait for "new sample ready" (bit2)
        while (self._read8(_VL6180X_REG_RESULT_INTERRUPT_STATUS_GPIO) & 0x04) == 0:
            pass
        # Read distance (mm)
        rng = self._read8(_VL6180X_REG_RESULT_RANGE_VAL)
        # Clear interrupts (range + ALS + error)
        self._write8(_VL6180X_REG_SYSTEM_INTERRUPT_CLEAR, 0x07)
        return rng

    def _load_settings(self):
        # Private settings (per app note)
        self._write8(0x0207, 0x01)
        self._write8(0x0208, 0x01)
        self._write8(0x0096, 0x00)
        self._write8(0x0097, 0xFD)
        self._write8(0x00E3, 0x00)
        self._write8(0x00E4, 0x04)
        self._write8(0x00E5, 0x02)
        self._write8(0x00E6, 0x01)
        self._write8(0x00E7, 0x03)
        self._write8(0x00F5, 0x02)
        self._write8(0x00D9, 0x05)
        self._write8(0x00DB, 0xCE)
        self._write8(0x00DC, 0x03)
        self._write8(0x00DD, 0xF8)
        self._write8(0x009F, 0x00)
        self._write8(0x00A3, 0x3C)
        self._write8(0x00B7, 0x00)
        self._write8(0x00BB, 0x3C)
        self._write8(0x00B2, 0x09)
        self._write8(0x00CA, 0x09)
        self._write8(0x0198, 0x01)
        self._write8(0x01B0, 0x17)
        self._write8(0x01AD, 0x00)
        self._write8(0x00FF, 0x05)
        self._write8(0x0100, 0x05)
        self._write8(0x0199, 0x05)
        self._write8(0x01A6, 0x1B)
        self._write8(0x01AC, 0x3E)
        self._write8(0x01A7, 0x1F)
        self._write8(0x0030, 0x00)
        # Recommended public registers
        self._write8(0x0011, 0x10)
        self._write8(0x010A, 0x30)
        self._write8(0x003F, 0x46)
        self._write8(0x0031, 0xFF)
        self._write8(0x0040, 0x63)
        self._write8(0x002E, 0x01)
        # Optional public
        self._write8(0x001B, 0x09)  # ~100ms inter-measure (nominal)
        self._write8(0x003E, 0x31)  # ALS inter-measure ~500ms
        self._write8(0x0014, 0x24)  # Interrupt config

    # --- Low-level I2C helpers (16-bit reg, 8/16-bit data) ---
    def _write8(self, reg, val):
        self._i2c.writeto_mem(self._addr, reg, bytes([val & 0xFF]), addrsize=16)

    def _write16(self, reg, val):
        self._i2c.writeto_mem(self._addr, reg, bytes([(val >> 8) & 0xFF, val & 0xFF]), addrsize=16)

    def _read8(self, reg):
        return int.from_bytes(self._i2c.readfrom_mem(self._addr, reg, 1, addrsize=16), "big")

    def _read16(self, reg):
        data = self._i2c.readfrom_mem(self._addr, reg, 2, addrsize=16)
        return (data[0] << 8) | data[1]