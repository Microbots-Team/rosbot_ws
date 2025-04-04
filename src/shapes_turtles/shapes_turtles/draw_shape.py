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
from turtlesim.srv import SetPen


PALETTE = [
    (255, 0, 0),  # Red
    (0, 255, 0),  # Green
    (0, 0, 255),  # Blue
    (255, 255, 0),  # Yellow
    (255, 0, 255),  # Magenta
    (0, 255, 255),  # Cyan
    (255, 255, 255),  # White
    (0, 0, 0),  # Black
]


class DrawNodeShape:
    def __init__(self):
        self.node = Node("draw_shape")
        self.logger = self.node.get_logger()
        self.logger.set_level(LoggingSeverity.DEBUG)

        # -- Publishers
        self.cmd_vel = self.node.create_publisher(Twist, "/turtle1/cmd_vel", 5)

        # -- Services Clients
        self.srv_set_pen = self.node.create_client(SetPen, "/turtle1/set_pen")

        # -- Actions Clients
        self.act_rotate_absolute = ActionClient(
            self.node, RotateAbsolute, "/turtle1/rotate_absolute"
        )

        # -- Program
        self.logger.info("Program started.")
        self.program()
        self.logger.info("Program finished.")

    def program(self) -> None:
        shape_id = 0

        for vertices in [3, 5, 9]:
            for speed in [1, 2]:
                self.set_pen(PALETTE[shape_id % len(PALETTE)])
                self.draw_shape(vertices, speed)
                shape_id += 1

    def draw_shape(self, vertices: int, speed=1.0) -> None:
        self.logger.info(
            f"Drawing shape of {vertices} vertices with speed of {speed}..."
        )
        vertex_angle = math.pi * 2 / vertices

        for i in range(vertices):
            self.rotate_absolute(i * vertex_angle)
            self.move(linear=speed)
            sleep(1)
        self.stop()

    def set_pen(
        self, color: tuple[int, int, int] = (1, 1, 1), width=3, active=True
    ) -> None:
        req = SetPen.Request()
        req.r, req.g, req.b = color
        req.width = width
        req.off = int(not active)

        res_future = self.srv_set_pen.call_async(req)
        rclpy.spin_until_future_complete(self.node, res_future)
        res_future.result()

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


def main(args=None):
    with suppress(KeyboardInterrupt):
        rclpy.init(args=args)
        DrawNodeShape()
        rclpy.shutdown()
    print()


if __name__ == "__main__":
    main()
