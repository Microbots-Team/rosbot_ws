
## Development Journal

1. Create the package using `ros2 pkg create --built-type ament_python --license MIT shapes_turtles`.
2. Set the project version, description, maintainer details in both `package.xml` and `setup.py`.
3. Start a `turtlesim` instance using `ros2 run turtlesim turtlesim_node` (in a background terminal).
    - If you forgot the exact name of a package, and it's executables names, you can check them through `ros2 pkg list`
    and `ros2 pkg executables <pkg-name>`.
4. Explore the available system of nodes, topics, services, etc...
    - To view the current nodes list: `ros2 node list`, we should see the `/turtlesim` node.
    - To view the current topics list: `ros2 topic list`, there should be the following:
        - `/parameter_events` and `/rosout` (uninteresting).
        - `/turtle1/cmd_vel`: Allows us to move the turtle, we would write to (publish messages to).
        - _(ask students to guess before revealing the answers of the ones below)_
        - `/turtle1/color_sensor`: Probably reports the color of the current pixel under the turtle's center, would be used to read from (subscribe to messages from). But it's not useful for our application.
        - `/turtle1/pose`: Probably reports the current position and orientation of the turtle. Might be useful for tracking the progress of the robot motion. Would be used for reading from. Maybe even writing to? Depends on the simulation if it supports so.
    - To view the current services list: `ros2 service list`.
    - To view the current actions list: `ros2 action list`.
5. Check how the turtles can be controlled.
    - We can do so by using the `turtle_teleop_key` node for control using keyboard, and listening/inspecting the messages on the `/turtle1/cmd_vel` topic.
        - `ros2 run turtlesim turtle_teleop_key`.
        - `ros2 topic echo /turtle1/cmd_vel`.
    - We can check the message types of topics, services and actions ("interfaces").
        1. Get type name through `ros2 topic type <topic>`, `ros2 service type <server>`, or `ros2 action type <action>`.
            - `ros2 topic type /turtle1/cmd_vel` ⇒ `geometry_msgs/msg/Twist`.
            - ~~`ros2 action type /turtle1/rotate_absolute` ⇒ `turtlesim/action/RotateAbsolute`.~~ (leave actions for a later section)
        2. Get type details through `ros2 interface show <type>`.
            - `ros2 interface show geometry_msgs/msg/Twist`.
            - ~~`ros2 interface show turtlesim/action/RotateAbsolute`.~~ (leave actions for a later section)
                - For actions, the structs come in the following order: request, response, feedback.
        3. Get example message using `ros2 interface proto <type>`.
            - `ros2 interface proto geometry_msgs/msg/Twist`.
    - We can use `rqt` for sending messages and call services manually using a GUI.
        - ⓘ A fun exercise would be using the `Robot Steering` plugin for controlling the turtle.
            - In-fact, a bunch of exercises can be done on the `rqt`.
6. Lets focus on making the robot move linearly.
    - Start by creating a python file, writing the boilerplate code, and defining the node executable in `setup.py`.
        - Make sure to add `<exec_depend>rclpy</exec_depend>` in `package.xml`.
        - A simple `Hello robots!` example with keyboard interrupt exception suppression.
    - Lets call our main node & executable `draw_shape`.
    - Build package using `colcon` and the symlink option:
        - `colcon build --symlink-install` (be sure to be in the root workspace directory)
    - Reactivate the workspace for the changes to take effect.
        - Reactivation is only needed when adding new executables to a package.
    - Now into putting the turtle into motion by code.
        - Add `<exec_depend>geometry_msgs</exec_depend>` to `package.xml` since we are going to use it.
        - Create a publisher, construct a message and send it.
7. Sequencing commands based on `time.sleep`.
    - Pack the message publishing code into a function for easier usage.

### Tips

- _Give tips on enabling VS Code auto-imports completion, type checker, auto-formatting with black, auto-saving._
- _Give tips on ordering imports and categorizing them (lookup the actual PEP I'm following)._
- For any ROS2 command, we can see the available sub-commands and options by executing it directly or with the `--help`/`-h` option.
    - `ros2`, `ros2 topic`, `ros2 -h`, `ros2 topic list --help`.
- **Always name your packages and python files using underscores (`_`), not hyphens (`-`)!**
- Messages, Servers, etc... types are defined in their own packages and need building because they generate wrappers for each supported programming language (Python & C++).
- We can notice common structures in standard packages.
    - There are packages containing types definitions only, their name usually ends with `_msgs`, like `geometry_msgs`.
    - The types definitions are usually contained in the sub-directories `msg` and `srv` within packages.
    - Robots packages usually come with a `_teleop_key` node for testing the robot using manual control.
- If `rqt` got messed up,

### Further Reading

- Interesting article on actions: https://design.ros2.org/articles/actions.html, it has **useful state diagrams**.