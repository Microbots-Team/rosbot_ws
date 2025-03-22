import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from gpiozero import Button
import time

class LineFollowerSensorNode(Node):
    def __init__(self):
        super().__init__('line_follower_sensor_node')

        # GPIO setup using gpiozero
        self.sensor_pin = 14  # GPIO 14 (BCM pin)
        self.sensor = Button(self.sensor_pin)  # Use gpiozero Button class

        # Publisher setup
        self.publisher_ = self.create_publisher(Bool, 'line_follower_status', 10)

        # Timer to continuously read the sensor data
        self.timer = self.create_timer(0.1, self.timer_callback)  # 0.1s interval (10 Hz)

    def timer_callback(self):
        # Read the GPIO pin state using gpiozero
        sensor_state = self.sensor.is_pressed  # True if pressed (line detected), False if not pressed (no line)

        # Log the sensor state
        self.get_logger().info(f'Line Follower Sensor State: {"On Line" if sensor_state else "Off Line"}')

        # Publish the sensor data to the topic
        msg = Bool()
        msg.data = sensor_state
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    line_follower_sensor_node = LineFollowerSensorNode()

    rclpy.spin(line_follower_sensor_node)

    # Clean up GPIO when shutting down
    line_follower_sensor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
