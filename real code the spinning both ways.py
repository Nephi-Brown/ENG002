from machine import Pin,PWM
import time


#defining motorboard pins
dir1 = Pin(17,Pin.OUT)
pwm1=PWM(Pin(16))
pwm1.freq(1000)#sets the pwm for 1000 Hz(turns it on and off 1000 times a second)
while True:
   dir1.value(1)
   pwm1.duty_u16(32768)#this is 50 percent as the range is 0-65535
   time.sleep(2)
   dir1.value(0)
   pwm1.duty_u16(16384)
   time.sleep(2)

