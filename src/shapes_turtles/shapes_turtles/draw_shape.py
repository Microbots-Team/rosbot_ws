from contextlib import suppress
from time import sleep

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class DrawNodeShape:
    def __init__(self):
        self.node = Node("draw_shape")
        self.logger = self.node.get_logger()

        # -- Publishers
        self.cmd_vel = self.node.create_publisher(Twist, "/turtle1/cmd_vel", 5)

        # -- Program
        self.logger.info("Program started.")
        self.program()
        self.logger.info("Program finished.")

    def program(self) -> None:
        for _ in range(6):
            self.move(linear=1.0)
            sleep(1)
            self.move(angular=1.0)
            sleep(1)
        self.stop()

    def move(self, *, linear=0.0, angular=0.0) -> None:
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular

        self.cmd_vel.publish(msg)

    def stop(self) -> None:
        self.move(linear=0.0, angular=0.0)

    def spin(self):
        # rclpy.spin(self.node)
        pass


def main(args=None):
    with suppress(KeyboardInterrupt):
        rclpy.init(args=args)
        DrawNodeShape().spin()
        rclpy.shutdown()
    print()


if __name__ == "__main__":
    main()
