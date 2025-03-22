import gpiod
import time

sensor_pin = 14
chip = gpiod.Chip('gpiochip0')
line = chip.get_line(sensor_pin)
line.request(consumer='sensor_test', type=gpiod.LINE_REQ_DIR_IN)

try:
    while True:
        sensor_state = line.get_value()
        print(f"Sensor state: {'On Line' if sensor_state else 'Off Line'}")
        time.sleep(1)
except KeyboardInterrupt:
    pass
