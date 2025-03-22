from gpiozero import Button
import time

sensor_pin = 4  # GPIO 4 (BCM pin)
sensor = Button(sensor_pin)

while True:
    if sensor.is_pressed:
        print("On Line")
    else:
        print("Off Line")
    time.sleep(0.5)
