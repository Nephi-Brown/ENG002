from machine import I2C, Pin
import time
from vl53l0x import VL53L0X
import ujason as cal #this is saving the data for another file

# Pico/Pico W default I2C0: SDA=GP4, SCL=GP5 (adjust if you wired differently)
i2c = I2C(1, sda=Pin(6), scl=Pin(7), freq=400_000)

# Create sensor (0x29 default). Add io_timeout_s if you want (e.g., 0.5)
tof = VL53L0X(i2c, address=0x29)

# Optional: continuous mode for faster reads
tof.start_continuous()
#place the centre at the known distance set inside the bracket
real_distances = [20, 50, 70]
#the number of measurements been taken at each point
sample_per_point = 80
delay = 0.05 #this is the time inbetween each sample point is taken

def measure_average(sample_per_point): #this takes in 'n' readings and then will return the average
    s=0 #this is our 'box' that will store our readings, is currently empty
    valid = 0 #this tells us how many of our readings are correct, starts at zero
    for y in range(sample_per_point): #this loops this part of the function for 'n' times, repeat the measuremt 'n' times
        d = tof.read_range() #d is now the variable holding the distance measured by the tof,stores the distance
        if d and d > 0: #this first checks that d is not zero or none, as if d in python checks if d is 'truth' in boolen context, and d>0 checks its postive if the first d is correct
           s +=d #this is adding whatever our d value is to our box s
           valid +=1 #if all above is true adds 1 to our valid box, this is for avergering later
        time.sleep(delay)
    return(s/valid) if valid else None #once the loop is finsiehd it will now find the averge is if valid is true 

def measure_the_error(real_distance, sample_per_point):#this function is for correcting the error that the sensor reads
    measured = measure_average(sample_per_point)
    if measured is None:#the is function checks the idenity of measured, if its none that it will stop
        return None #this makes it stop
    error = measured - real_distance #this calcuates the error
    print ("measured:",measured "real_distance:",real_distance "error:",error) #this prints the data we have collected
    return error
calibration_data = {} #this is an empty libaray where we will eveuntaully store the calibration code
for dist in real_distance #in each loop the dist vairable will get a number from real_distance
error = measure_the_error(dist, sample_per_point) # this is storing the cailbration in error, calling the cailration function(can use error again as the previous error is a local variable to that function, not like this one which is global)
calibration_data[dist] = error #now putting the value of error in our data set, the dist is the real number, like the label, eg 20(real number) error 2
cal.save("calibration_data.json", calibration_data)#this saves the calibrated data unto anothe file so it can be used later

