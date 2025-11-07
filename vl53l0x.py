# MicroPython driver for VL53L0X (ported from Adafruit CircuitPython version)
# - No external libs required (uses machine.I2C)
# - Compatible with Raspberry Pi Pico / Pico W
#
# SPDX-License-Identifier: MIT
# Original author: Tony DiCola for Adafruit Industries
# MicroPython port: ChatGPT (2025)

import math
import time
from machine import I2C

try:
    from micropython import const
except ImportError:
    def const(x): return x

__version__ = "0.0.0-micropython"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_VL53L0X.git"

# ---- Configuration constants (same as CircuitPython driver) ----
_SYSRANGE_START = const(0x00)
_SYSTEM_THRESH_HIGH = const(0x0C)
_SYSTEM_THRESH_LOW = const(0x0E)
_SYSTEM_SEQUENCE_CONFIG = const(0x01)
_SYSTEM_RANGE_CONFIG = const(0x09)
_SYSTEM_INTERMEASUREMENT_PERIOD = const(0x04)
_SYSTEM_INTERRUPT_CONFIG_GPIO = const(0x0A)
_GPIO_HV_MUX_ACTIVE_HIGH = const(0x84)
_SYSTEM_INTERRUPT_CLEAR = const(0x0B)
_RESULT_INTERRUPT_STATUS = const(0x13)
_RESULT_RANGE_STATUS = const(0x14)
_RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN = const(0xBC)
_RESULT_CORE_RANGING_TOTAL_EVENTS_RTN = const(0xC0)
_RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF = const(0xD0)
_RESULT_CORE_RANGING_TOTAL_EVENTS_REF = const(0xD4)
_RESULT_PEAK_SIGNAL_RATE_REF = const(0xB6)
_ALGO_PART_TO_PART_RANGE_OFFSET_MM = const(0x28)
_I2C_SLAVE_DEVICE_ADDRESS = const(0x8A)
_MSRC_CONFIG_CONTROL = const(0x60)
_PRE_RANGE_CONFIG_MIN_SNR = const(0x27)
_PRE_RANGE_CONFIG_VALID_PHASE_LOW = const(0x56)
_PRE_RANGE_CONFIG_VALID_PHASE_HIGH = const(0x57)
_PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT = const(0x64)
_FINAL_RANGE_CONFIG_MIN_SNR = const(0x67)
_FINAL_RANGE_CONFIG_VALID_PHASE_LOW = const(0x47)
_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH = const(0x48)
_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = const(0x44)
_PRE_RANGE_CONFIG_SIGMA_THRESH_HI = const(0x61)
_PRE_RANGE_CONFIG_SIGMA_THRESH_LO = const(0x62)
_PRE_RANGE_CONFIG_VCSEL_PERIOD = const(0x50)
_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI = const(0x51)
_PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO = const(0x52)
_SYSTEM_HISTOGRAM_BIN = const(0x81)
_HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT = const(0x33)
_HISTOGRAM_CONFIG_READOUT_CTRL = const(0x55)
_FINAL_RANGE_CONFIG_VCSEL_PERIOD = const(0x70)
_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI = const(0x71)
_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO = const(0x72)
_CROSSTALK_COMPENSATION_PEAK_RATE_MCPS = const(0x20)
_MSRC_CONFIG_TIMEOUT_MACROP = const(0x46)
_SOFT_RESET_GO2_SOFT_RESET_N = const(0xBF)
_IDENTIFICATION_MODEL_ID = const(0xC0)
_IDENTIFICATION_REVISION_ID = const(0xC2)
_OSC_CALIBRATE_VAL = const(0xF8)
_GLOBAL_CONFIG_VCSEL_WIDTH = const(0x32)
_GLOBAL_CONFIG_SPAD_ENABLES_REF_0 = const(0xB0)
_GLOBAL_CONFIG_SPAD_ENABLES_REF_1 = const(0xB1)
_GLOBAL_CONFIG_SPAD_ENABLES_REF_2 = const(0xB2)
_GLOBAL_CONFIG_SPAD_ENABLES_REF_3 = const(0xB3)
_GLOBAL_CONFIG_SPAD_ENABLES_REF_4 = const(0xB4)
_GLOBAL_CONFIG_SPAD_ENABLES_REF_5 = const(0xB5)
_GLOBAL_CONFIG_REF_EN_START_SELECT = const(0xB6)
_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD = const(0x4E)
_DYNAMIC_SPAD_REF_EN_START_OFFSET = const(0x4F)
_POWER_MANAGEMENT_GO1_POWER_FORCE = const(0x80)
_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV = const(0x89)
_ALGO_PHASECAL_LIM = const(0x30)
_ALGO_PHASECAL_CONFIG_TIMEOUT = const(0x30)
_VCSEL_PERIOD_PRE_RANGE = const(0)
_VCSEL_PERIOD_FINAL_RANGE = const(1)

# ---- Utility conversions (same math as CP version) ----
def _decode_timeout(val: int) -> float:
    return float(val & 0xFF) * math.pow(2.0, ((val & 0xFF00) >> 8)) + 1

def _encode_timeout(timeout_mclks: float) -> int:
    timeout_mclks = int(timeout_mclks) & 0xFFFF
    if timeout_mclks <= 0:
        return 0
    ls_byte = timeout_mclks - 1
    ms_byte = 0
    while ls_byte > 255:
        ls_byte >>= 1
        ms_byte += 1
    return ((ms_byte << 8) | (ls_byte & 0xFF)) & 0xFFFF

def _macro_period_ns(vcsel_period_pclks: int) -> int:
    # matches CP implementation
    return ((2304 * (vcsel_period_pclks) * 1655) + 500) // 1000

def _timeout_mclks_to_microseconds(timeout_period_mclks: int, vcsel_period_pclks: int) -> int:
    macro_period_ns = _macro_period_ns(vcsel_period_pclks)
    return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns // 2)) // 1000

def _timeout_microseconds_to_mclks(timeout_period_us: int, vcsel_period_pclks: int) -> int:
    macro_period_ns = _macro_period_ns(vcsel_period_pclks)
    return ((timeout_period_us * 1000) + (macro_period_ns // 2)) // macro_period_ns

# ---- Time helpers for MicroPython ----
def _ticks_s():
    # use milliseconds to avoid float drift
    return time.ticks_ms()

def _elapsed_s(start_ms, timeout_s):
    # returns True if elapsed >= timeout_s
    return time.ticks_diff(time.ticks_ms(), start_ms) >= int(timeout_s * 1000)

# ---- Low-level I2C helpers (no bus device dependency) ----
class _I2CDev:
    def __init__(self, i2c: I2C, addr: int):
        self.i2c = i2c
        self.addr = addr

    def write_u8(self, reg: int, val: int):
        self.i2c.writeto(self.addr, bytes([reg & 0xFF, val & 0xFF]))

    def write_u16_be(self, reg: int, val: int):
        self.i2c.writeto(self.addr, bytes([reg & 0xFF, (val >> 8) & 0xFF, val & 0xFF]))

    def write_bytes(self, reg: int, buf: bytes):
        # reg followed by payload
        self.i2c.writeto(self.addr, bytes([reg & 0xFF]) + bytes(buf))

    def read_u8(self, reg: int) -> int:
        self.i2c.writeto(self.addr, bytes([reg & 0xFF]), False)
        b = self.i2c.readfrom(self.addr, 1)
        return b[0]

    def read_u16_be(self, reg: int) -> int:
        self.i2c.writeto(self.addr, bytes([reg & 0xFF]), False)
        b = self.i2c.readfrom(self.addr, 2)
        return (b[0] << 8) | b[1]

    def read_bytes(self, reg: int, n: int) -> bytearray:
        self.i2c.writeto(self.addr, bytes([reg & 0xFF]), False)
        return bytearray(self.i2c.readfrom(self.addr, n))

class VL53L0X:
    """MicroPython driver for the VL53L0X distance sensor."""
    _continuous_mode = False

    def __init__(self, i2c: I2C, address: int = 0x29, io_timeout_s: float = 0.0):
        self._i2c = i2c
        self._addr = address
        self._dev = _I2CDev(i2c, address)
        self.io_timeout_s = float(io_timeout_s)
        self._data_ready = False

        # Check ID registers
        if (self._read_u8(0xC0) != 0xEE or
            self._read_u8(0xC1) != 0xAA or
            self._read_u8(0xC2) != 0x10):
            raise RuntimeError("VL53L0X not found. Check wiring/I2C address.")

        # Init sequence (Pololu-based, matches CP lib)
        for a,v in ((0x88,0x00),(0x80,0x01),(0xFF,0x01),(0x00,0x00)):
            self._write_u8(a,v)
        self._stop_variable = self._read_u8(0x91)
        for a,v in ((0x00,0x01),(0xFF,0x00),(0x80,0x00)):
            self._write_u8(a,v)

        # Disable certain limit checks
        cfg = self._read_u8(_MSRC_CONFIG_CONTROL) | 0x12
        self._write_u8(_MSRC_CONFIG_CONTROL, cfg)

        # Set final range signal rate limit to 0.25 MCPS
        self.signal_rate_limit = 0.25

        self._write_u8(_SYSTEM_SEQUENCE_CONFIG, 0xFF)
        spad_count, spad_is_aperture = self._get_spad_info()

        # Read SPAD enable map
        ref_spad_map = self._dev.read_bytes(_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, 7)

        for a,v in (
            (0xFF,0x01),
            (_DYNAMIC_SPAD_REF_EN_START_OFFSET,0x00),
            (_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD,0x2C),
            (0xFF,0x00),
            (_GLOBAL_CONFIG_REF_EN_START_SELECT,0xB4),
        ):
            self._write_u8(a,v)

        first_spad_to_enable = 12 if spad_is_aperture else 0
        spads_enabled = 0
        # Build modified map in place (indexing ref_spad_map[0..6])
        for i in range(48):
            byte_i = 1 + (i // 8)  # in CP they had leading address byte; here we read pure data
            bit = 1 << (i % 8)
            if i < first_spad_to_enable or spads_enabled == spad_count:
                ref_spad_map[i // 8] &= ~bit
            elif (ref_spad_map[i // 8] >> (i % 8)) & 0x1:
                spads_enabled += 1

        # Write back the 6 bytes starting at REF_0 (we read 7 to match CP, but 6/7 both okay)
        self._dev.write_bytes(_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map)

        for a,v in (
            (0xFF,0x01),(0x00,0x00),(0xFF,0x00),(0x09,0x00),(0x10,0x00),(0x11,0x00),
            (0x24,0x01),(0x25,0xFF),(0x75,0x00),(0xFF,0x01),(0x4E,0x2C),(0x48,0x00),
            (0x30,0x20),(0xFF,0x00),(0x30,0x09),(0x54,0x00),(0x31,0x04),(0x32,0x03),
            (0x40,0x83),(0x46,0x25),(0x60,0x00),(0x27,0x00),(0x50,0x06),(0x51,0x00),
            (0x52,0x96),(0x56,0x08),(0x57,0x30),(0x61,0x00),(0x62,0x00),(0x64,0x00),
            (0x65,0x00),(0x66,0xA0),(0xFF,0x01),(0x22,0x32),(0x47,0x14),(0x49,0xFF),
            (0x4A,0x00),(0xFF,0x00),(0x7A,0x0A),(0x7B,0x00),(0x78,0x21),(0xFF,0x01),
            (0x23,0x34),(0x42,0x00),(0x44,0xFF),(0x45,0x26),(0x46,0x05),(0x40,0x40),
            (0x0E,0x06),(0x20,0x1A),(0x43,0x40),(0xFF,0x00),(0x34,0x03),(0x35,0x44),
            (0xFF,0x01),(0x31,0x04),(0x4B,0x09),(0x4C,0x05),(0x4D,0x04),(0xFF,0x00),
            (0x44,0x00),(0x45,0x20),(0x47,0x08),(0x48,0x28),(0x67,0x00),(0x70,0x04),
            (0x71,0x01),(0x72,0xFE),(0x76,0x00),(0x77,0x00),(0xFF,0x01),(0x0D,0x01),
            (0xFF,0x00),(0x80,0x01),(0x01,0xF8),(0xFF,0x01),(0x8E,0x01),(0x00,0x01),
            (0xFF,0x00),(0x80,0x00),
        ):
            self._write_u8(a,v)

        self._write_u8(_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04)
        val = self._read_u8(_GPIO_HV_MUX_ACTIVE_HIGH)
        self._write_u8(_GPIO_HV_MUX_ACTIVE_HIGH, val & ~0x10)  # active low
        self._write_u8(_SYSTEM_INTERRUPT_CLEAR, 0x01)

        self._measurement_timing_budget_us = self.measurement_timing_budget
        self._write_u8(_SYSTEM_SEQUENCE_CONFIG, 0xE8)
        self.measurement_timing_budget = self._measurement_timing_budget_us
        self._write_u8(_SYSTEM_SEQUENCE_CONFIG, 0x01)
        self._perform_single_ref_calibration(0x40)
        self._write_u8(_SYSTEM_SEQUENCE_CONFIG, 0x02)
        self._perform_single_ref_calibration(0x00)
        self._write_u8(_SYSTEM_SEQUENCE_CONFIG, 0xE8)

    # ---- Low-level reg access ----
    def _read_u8(self, reg: int) -> int:
        return self._dev.read_u8(reg)

    def _read_u16(self, reg: int) -> int:
        return self._dev.read_u16_be(reg)

    def _write_u8(self, reg: int, val: int):
        self._dev.write_u8(reg, val)

    def _write_u16(self, reg: int, val: int):
        self._dev.write_u16_be(reg, val)

    # ---- SPAD / calibration ----
    def _get_spad_info(self):
        for a,v in ((0x80,0x01),(0xFF,0x01),(0x00,0x00),(0xFF,0x06)):
            self._write_u8(a,v)
        self._write_u8(0x83, self._read_u8(0x83) | 0x04)
        for a,v in ((0xFF,0x07),(0x81,0x01),(0x80,0x01),(0x94,0x6B),(0x83,0x00)):
            self._write_u8(a,v)

        start = _ticks_s()
        while self._read_u8(0x83) == 0x00:
            if self.io_timeout_s > 0 and _elapsed_s(start, self.io_timeout_s):
                raise RuntimeError("Timeout waiting for VL53L0X (SPAD info).")

        self._write_u8(0x83, 0x01)
        tmp = self._read_u8(0x92)
        count = tmp & 0x7F
        is_aperture = ((tmp >> 7) & 0x01) == 1

        for a,v in ((0x81,0x00),(0xFF,0x06)):
            self._write_u8(a,v)
        self._write_u8(0x83, self._read_u8(0x83) & ~0x04)
        for a,v in ((0xFF,0x01),(0x00,0x01),(0xFF,0x00),(0x80,0x00)):
            self._write_u8(a,v)
        return count, is_aperture

    def _perform_single_ref_calibration(self, vhv_init_byte: int):
        self._write_u8(_SYSRANGE_START, (0x01 | (vhv_init_byte & 0xFF)) & 0xFF)
        start = _ticks_s()
        while (self._read_u8(_RESULT_INTERRUPT_STATUS) & 0x07) == 0:
            if self.io_timeout_s > 0 and _elapsed_s(start, self.io_timeout_s):
                raise RuntimeError("Timeout during ref calibration.")
        self._write_u8(_SYSTEM_INTERRUPT_CLEAR, 0x01)
        self._write_u8(_SYSRANGE_START, 0x00)

    # ---- VCSEL / timing ----
    def _get_vcsel_pulse_period(self, vcsel_period_type: int) -> int:
        if vcsel_period_type == _VCSEL_PERIOD_PRE_RANGE:
            val = self._read_u8(_PRE_RANGE_CONFIG_VCSEL_PERIOD)
            return (((val) + 1) & 0xFF) << 1
        elif vcsel_period_type == _VCSEL_PERIOD_FINAL_RANGE:
            val = self._read_u8(_FINAL_RANGE_CONFIG_VCSEL_PERIOD)
            return (((val) + 1) & 0xFF) << 1
        return 255

    def _get_sequence_step_enables(self):
        seq = self._read_u8(_SYSTEM_SEQUENCE_CONFIG)
        tcc = ((seq >> 4) & 0x1) > 0
        dss = ((seq >> 3) & 0x1) > 0
        msrc = ((seq >> 2) & 0x1) > 0
        pre_range = ((seq >> 6) & 0x1) > 0
        final_range = ((seq >> 7) & 0x1) > 0
        return tcc, dss, msrc, pre_range, final_range

    def _get_sequence_step_timeouts(self, pre_range: int):
        pre_range_vcsel = self._get_vcsel_pulse_period(_VCSEL_PERIOD_PRE_RANGE)
        msrc_dss_tcc_mclks = (self._read_u8(_MSRC_CONFIG_TIMEOUT_MACROP) + 1) & 0xFF
        msrc_dss_tcc_us = _timeout_mclks_to_microseconds(msrc_dss_tcc_mclks, pre_range_vcsel)
        pre_range_mclks = _decode_timeout(self._read_u16(_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI))
        pre_range_us = _timeout_mclks_to_microseconds(int(pre_range_mclks), pre_range_vcsel)

        final_range_vcsel = self._get_vcsel_pulse_period(_VCSEL_PERIOD_FINAL_RANGE)
        final_range_mclks = _decode_timeout(self._read_u16(_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI))
        if pre_range:
            final_range_mclks -= pre_range_mclks
        final_range_us = _timeout_mclks_to_microseconds(int(final_range_mclks), final_range_vcsel)
        return (msrc_dss_tcc_us, pre_range_us, final_range_us, final_range_vcsel, int(pre_range_mclks))

    # ---- Public properties ----
    @property
    def signal_rate_limit(self) -> float:
        val = self._read_u16(_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT)
        return val / (1 << 7)

    @signal_rate_limit.setter
    def signal_rate_limit(self, val: float):
        if not (0.0 <= val <= 511.99):
            raise ValueError("signal_rate_limit out of range")
        self._write_u16(_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, int(val * (1 << 7)))

    @property
    def measurement_timing_budget(self) -> int:
        budget_us = 1910 + 960
        tcc, dss, msrc, pre_range, final_range = self._get_sequence_step_enables()
        msrc_dss_tcc_us, pre_range_us, final_range_us, _, _ = self._get_sequence_step_timeouts(pre_range)
        if tcc:
            budget_us += msrc_dss_tcc_us + 590
        if dss:
            budget_us += 2 * (msrc_dss_tcc_us + 690)
        elif msrc:
            budget_us += msrc_dss_tcc_us + 660
        if pre_range:
            budget_us += pre_range_us + 660
        if final_range:
            budget_us += final_range_us + 550
        self._measurement_timing_budget_us = budget_us
        return budget_us

    @measurement_timing_budget.setter
    def measurement_timing_budget(self, budget_us: int):
        if budget_us < 20000:
            raise ValueError("budget must be >= 20000 us")
        used = 1320 + 960
        tcc, dss, msrc, pre_range, final_range = self._get_sequence_step_enables()
        msrc_dss_tcc_us, pre_range_us, _, final_vcsel, pre_range_mclks = self._get_sequence_step_timeouts(pre_range)
        if tcc:
            used += msrc_dss_tcc_us + 590
        if dss:
            used += 2 * (msrc_dss_tcc_us + 690)
        elif msrc:
            used += msrc_dss_tcc_us + 660
        if pre_range:
            used += pre_range_us + 660
        if final_range:
            used += 550
            if used > budget_us:
                raise ValueError("Requested timeout too big.")
            final_us = budget_us - used
            final_mclks = _timeout_microseconds_to_mclks(final_us, final_vcsel)
            if pre_range:
                final_mclks += pre_range_mclks
            self._write_u16(_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, _encode_timeout(final_mclks))
            self._measurement_timing_budget_us = budget_us

    @property
    def data_ready(self) -> bool:
        if not self._data_ready:
            self._data_ready = (self._read_u8(_RESULT_INTERRUPT_STATUS) & 0x07) != 0
        return self._data_ready

    @property
    def is_continuous_mode(self) -> bool:
        return self._continuous_mode

    # ---- High-level range APIs ----
    @property
    def range(self) -> int:
        if not self._continuous_mode:
            self.do_range_measurement()
        return self.read_range()

    @property
    def distance(self) -> float:
        return self.range / 10.0  # cm

    def do_range_measurement(self):
        for a,v in (
            (0x80,0x01),(0xFF,0x01),(0x00,0x00),(0x91,self._stop_variable),(0x00,0x01),
            (0xFF,0x00),(0x80,0x00),(_SYSRANGE_START,0x01),
        ):
            self._write_u8(a,v)
        start = _ticks_s()
        while (self._read_u8(_SYSRANGE_START) & 0x01) > 0:
            if self.io_timeout_s > 0 and _elapsed_s(start, self.io_timeout_s):
                raise RuntimeError("Timeout starting range measurement.")

    def read_range(self) -> int:
        start = _ticks_s()
        while not self.data_ready:
            if self.io_timeout_s > 0 and _elapsed_s(start, self.io_timeout_s):
                raise RuntimeError("Timeout waiting for data_ready.")
        # Range result is at RESULT_RANGE_STATUS + 10 (big-endian)
        rng = self._read_u16(_RESULT_RANGE_STATUS + 10)
        self._write_u8(_SYSTEM_INTERRUPT_CLEAR, 0x01)
        self._data_ready = False
        return rng

    # ---- Continuous mode ----
    def continuous_mode(self):
        return self

    def __enter__(self):
        self.start_continuous()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.stop_continuous()

    def start_continuous(self):
        for a,v in (
            (0x80,0x01),(0xFF,0x01),(0x00,0x00),(0x91,self._stop_variable),(0x00,0x01),
            (0xFF,0x00),(0x80,0x00),(_SYSRANGE_START,0x02),
        ):
            self._write_u8(a,v)
        start = _ticks_s()
        while (self._read_u8(_SYSRANGE_START) & 0x01) > 0:
            if self.io_timeout_s > 0 and _elapsed_s(start, self.io_timeout_s):
                raise RuntimeError("Timeout entering continuous mode.")
        self._continuous_mode = True

    def stop_continuous(self):
        for a,v in (
            (_SYSRANGE_START,0x01),(0xFF,0x01),(0x00,0x00),(0x91,0x00),(0x00,0x01),(0xFF,0x00),
        ):
            self._write_u8(a,v)
        self._continuous_mode = False
        # Return to single ranging mode sanity
        self.do_range_measurement()

    # ---- Address change ----
    def set_address(self, new_address: int):
        """Set new 7-bit address (0x08..0x77 typical). Call with other sensors held in reset."""
        self._write_u8(_I2C_SLAVE_DEVICE_ADDRESS, new_address & 0x7F)
        # Update local device reference
        self._addr = new_address & 0x7F
        self._dev = _I2CDev(self._i2c, self._addr)