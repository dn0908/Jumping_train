import RPi.GPIO as IO
import time
from gpiozero import Motor

motor1 = Motor(12, 18)
motor2 = Motor(13, 19)

while True:
    motor1.forward(1)
    motor2.backward(1)
    time.sleep(5)

    motor1.stop()
    motor2.stop()

    time.sleep(3)