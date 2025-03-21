from contextlib import suppress

import rclpy
from rclpy.node import Node


class DrawNodeShape:
    def __init__(self):
        self.node = Node("draw_shape")
        self.logger = self.node.get_logger()

        self.logger.info("Hello robots!")

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
