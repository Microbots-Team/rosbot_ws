from contextlib import suppress

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class DrawNodeShape:
    def __init__(self):
        self.node = Node("draw_shape")
        self.logger = self.node.get_logger()
        self.logger.info("Hello robots!")

        # -- Publishers
        self.cmd_vel = self.node.create_publisher(Twist, "/turtle1/cmd_vel", 5)

        # -- Program
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 0.5

        self.cmd_vel.publish(msg)
        self.logger.info("The turtle has been put into motion.")

    def spin(self):
        rclpy.spin(self.node)


def main(args=None):
    with suppress(KeyboardInterrupt):
        rclpy.init(args=args)
        DrawNodeShape().spin()
        rclpy.shutdown()
    print()


if __name__ == "__main__":
    main()
