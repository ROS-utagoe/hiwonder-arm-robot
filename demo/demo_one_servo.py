#!/usr/bin/python3
import pigpio
import time

pin = 12

pi = pigpio.pi()
pi.set_PWM_range(pin, 20000)
pi.set_PWM_frequency(pin, 50)
pi.set_PWM_dutycycle(pin, 1500)

while True:
    for i in range(500, 2500+1,5):
        pi.set_PWM_dutycycle(pin, i)
        time.sleep(0.01)
    time.sleep(0.5)
    for i in range(2500, 500-1,-5):
        pi.set_PWM_dutycycle(pin, i)
        time.sleep(0.01)
    time.sleep(0.5)
