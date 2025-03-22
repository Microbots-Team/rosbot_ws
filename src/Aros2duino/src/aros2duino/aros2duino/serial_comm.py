import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

import serial
import time

serial_comm = serial.Serial()

class CommunicationNode(Node):
    
    def __init__(self):
        super().__init__("serial_comm")

        self.declare_parameter("serial_port", "None")
        self.declare_parameter("baud_rate", 0)
        self.declare_parameter("subscribe_to", "/cmd_vel")  # Now default to cmd_vel!

        sp, br, sub = self.read_parameters()

        self.connect_serial_port(serial_port=sp, baud_rate=br)
        time.sleep(0.5)

        self.data_publisher_ = self.create_publisher(String, "serial_comm", 10)  # Serial data publisher

        # Subscribe to Twist messages
        self.data_subscriber_ = self.create_subscription(Twist, sub, self.twist_callback, 10)

        self.get_logger().info("Serial communication has started.")
        self.timer_ = self.create_timer(0.1, self.read_serial_data)  # Timer for reading serial

    def read_parameters(self):
        serial_port_ = self.get_parameter("serial_port").get_parameter_value().string_value
        baud_rate_ = self.get_parameter("baud_rate").get_parameter_value().integer_value
        subscribe_ = self.get_parameter("subscribe_to").get_parameter_value().string_value

        self.get_logger().info(f"Port: {serial_port_} Baud Rate: {baud_rate_} Subscribed To: {subscribe_}")
        return serial_port_, baud_rate_, subscribe_

    def connect_serial_port(self, serial_port, baud_rate):
        serial_comm.port = serial_port
        serial_comm.baudrate = baud_rate
        serial_comm.timeout = 1
        serial_comm.open()

    def read_serial_data(self):
        try:
            msg = String()
            msg.data = serial_comm.readline().decode("utf-8").rstrip("\n").rstrip("\r")
            if msg.data:  # Avoid publishing empty lines
                self.data_publisher_.publish(msg)
        except Exception as e:
            self.get_logger().warn("Serial communication error. -Reading")
            self.get_logger().warn(repr(e))

    # ✅ This is the new callback for Twist messages
    def twist_callback(self, msg: Twist):
        try:
            # Convert Twist message to string like: linear.x:0.5,angular.z:0.5
            formatted = f"linear.x:{msg.linear.x},angular.z:{msg.angular.z}\n"
            serial_comm.write(formatted.encode("utf-8"))
            self.get_logger().info(f"Sent: {formatted}")
        except Exception as e:
            self.get_logger().warn("Serial communication error. -Writing Twist")
            self.get_logger().warn(repr(e))


def main(args=None):
    rclpy.init(args=args)
    node = CommunicationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
