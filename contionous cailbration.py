from machine import I2C, Pin
import time
from vl53l0x import VL53L0X
# Pico/Pico W default I2C0: SDA=GP4, SCL=GP5 (adjust if you wired differently)
i2c = I2C(1, sda=Pin(6), scl=Pin(7), freq=400_000)

# Create sensor (0x29 default). Add io_timeout_s if you want (e.g., 0.5)
tof = VL53L0X(i2c, address=0x29)

# Optional: continuous mode for faster reads
tof.start_continuous()

sensor_buffer = []#empty list to store the data
buffer_size = 10 #how many measuremnets is measures at one time

def cailbrated_distance():
    dist = tof.read_range()#this gets the distance measured from the tof sensor and asigns the value to the variable dist
    if dist and dist > 0:#making sure dist is true and postive, makes sure its 'valid'
        sensor_buffer.append(dist)#this is adding the dist variable to the end of our empty list
        if len(sensor_buffer) > buffer_size: #if the number of variables is bigger than the buffer size
            sensor_buffer.pop(0)#this then removes the first variable(the oldest variable) in the sensor buffer list
        average_distance = sum(sensor_buffer) /len(sensor_buffer)#sum adds all the variables, len counts how many variables there are
        return average_distance#this allows us to use the average value whenever the function is called
    else:
        return None
correction_factor = 0#starts the correction factor at 0, this is an esitmate in how often the sensor is constianlty off
y=0.1#how fast it does correction
def fixed_cailbration_distance(sensor_measure, real_measure):
    global correction_factor#this is calling a variable that is not defined inside the function,hence global
    if sensor_measure is not None:
            error = sensor_measure - real_measure#if this was the final correction value it would make the correction jump around eg if one was 52 one 54, the correction factor would be 2 and 4
            correction_factor = (1-y)*correction_factor+y*error#this is a standard low pass filter to smooth out the correction
            return correction_factor
        else:
            return correction_factor
while True:
    measurement = cailbrated_distance()
    if measurement is not None:
        distance_from_wall = 10#this is just the example distance from the wall, change this when measured
        actaul_distance = measurement - fixed_cailbration_distance(measurement,distance_from_wall)
    time.sleep(0.5)

    
    