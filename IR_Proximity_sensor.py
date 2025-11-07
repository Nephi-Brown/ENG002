from machine import Pin


sensor = Pin(15, Pin.IN)  #set GPIO-25 pin mode as INPUT

while True:
    if sensor.value() == 0:  #read the value of the sensor pin and compare it with 0
        print "object detected"
    else:
        print "no object"
 
    
   
 