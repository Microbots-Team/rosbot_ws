import math
from contextlib import suppress
from time import sleep

import rclpy
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from rclpy.task import Future
from geometry_msgs.msg import Twist
from turtlesim.action import RotateAbsolute


class DrawNodeShape:
    def __init__(self):
        self.node = Node("draw_shape")
        self.logger = self.node.get_logger()
        self.logger.set_level(LoggingSeverity.DEBUG)

        # -- Publishers
        self.cmd_vel = self.node.create_publisher(Twist, "/turtle1/cmd_vel", 5)

        # -- Action Clients
        self.act_rotate_absolute = ActionClient(
            self.node, RotateAbsolute, "/turtle1/rotate_absolute"
        )

        # -- Program
        self.logger.info("Program started.")
        self.program()
        self.logger.info("Program finished.")

    def program(self) -> None:
        # self.draw_shape(3)

        for i in [3, 5, 9]:
            for j in [1, 2]:
                self.draw_shape(i, j)

    def draw_shape(self, vertices: int, speed=1.0) -> None:
        self.logger.info(
            f"Drawing shape of {vertices} vertices with speed of {speed}..."
        )
        vertex_angle = math.pi * 2 / vertices

        for i in range(vertices):
            self.rotate_absolute(i * vertex_angle)
            self.move(linear=speed * 4)
            sleep(1 / 4)
        self.stop()

    def rotate_absolute(self, theta: float) -> None:
        goal = RotateAbsolute.Goal()
        goal.theta = theta

        self.logger.debug("Waiting for absolute rotation server to be ready..")
        self.act_rotate_absolute.wait_for_server()

        self.logger.debug(f"Rotating => {math.degrees(goal.theta):.1f}°")
        handle_future = self.act_rotate_absolute.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, handle_future)
        handle: ClientGoalHandle = handle_future.result()  # type: ignore

        assert handle.accepted, "Rotation rejected"

        result_future: Future = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        result_future.result()

        self.logger.debug("Rotated successfully.")

    def move(self, *, linear: int | float = 0.0, angular: int | float = 0.0) -> None:
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)

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
