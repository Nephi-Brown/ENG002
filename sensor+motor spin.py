from machine import Pin,PWM
from time import sleep

sensor = Pin(15,Pin.IN)
MR_PinB=Pin(18,Pin.OUT) #forward pin
MR_PinA=Pin(17,Pin.OUT) #backward pin
pwm1=PWM(Pin(19))
pwm1.freq(1000)#sets the pwm for 1000 Hz(turns it on and off 1000 times a second)

while True:
      if sensor.value() == 0:
         print ("object detected")
         MR_PinA.value(1)
         MR_PinB.value(0)
         pwm1.duty_u16(32768)#this is 50 percent as the range is 0-65535
      else:
         print ("no object")
         MR_PinA.value(0)
         MR_PinB.value(1)
         pwm1.duty_u16(32768)
      sleep(0.5)
