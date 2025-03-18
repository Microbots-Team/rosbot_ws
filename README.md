# rosbot_ws

ROS2 Workspace of the Romba-like robot.

## Getting started

1. Clone the repository locally on your ROS2 machine at the user home for convenience.

	```bash
	git clone https://github.com/Microbots-Team/rosbot_ws.git ~/rosbot_ws
	# or if you have write-access on the repository
	git clone git@github.com:Microbots-Team/rosbot_ws.git ~/rosbot_ws
	```

2. Activate your ROS2 installation.
3. Enter the cloned workspace directory: `cd ~/rosbot_ws`.
4. Run `colcon build --symlink-install`.
5. Activate the workspace overlay: `. ~/rosbot_ws/install/local_install.bash`.
6. Have a nice day!

### Creating additional packages

You can simply do so using the following commands:

```bash
cd ~/rosbot_ws/src
# for a python-based package:
ros2 pkg create --build-type ament_python --license MIT <pkg_name>
# for a cmake-based package (urdf and C++):
ros2 pkg create --build-type ament_cmake --license MIT <pkg_name>
```
