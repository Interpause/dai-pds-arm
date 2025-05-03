# Arm Control Stuff

There are three sets of arms: arm, arm2 and arm3. arm2 is the arm we are using.
arm is the old nested link design, arm3 is the last-ditch effort as light as possible
arm that we didn't use cause it still was not light enough.

## Adding New Arms

1. Install any Fusion to URDF exporter, I used: <https://github.com/bionicdl-sustech/ACDC4Robot>
2. Ensure that the Fusion CAD model is well-organized, namely:
  - Base component of robot must be named base_link and anchored.
  - All joints must be such that component 1 is the child component, and component 2 is the parent.
  - No nested components.
3. Use the exporter to export the Fusion CAD model as URDF.
4. Put the URDF into a ROS2 package, conventionally called `{some_name}_description`.
5. Use Moveit's Setup Assistant to configure the arm: <https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html>
6. Perform manual adjustment to joints as needed if the angular range is incorrect.

```sh
# Launch the setup assistant
ros2 launch moveit_setup_assistant setup_assistant.launch.py
# Check arm was configured correctly by moveit
ros2 launch arm_moveit_config demo.launch.py
```

## Controlling the Arm

Remember to replace the mock arm controller with the topic-based arm controller, see below for example:

<https://github.com/Interpause/dai-pds-arm/blob/ec006215c90d9d20ff83a97f44b97da54cf4b4fc/src/arm2_moveit_config/config/arm2.ros2_control.xacro#L7-L16>

Lauch the control software (Rviz):

```sh
ros2 launch arm_arduino_bridge ctrl.launch.py
```

## For Directly Setting Servo Values

```sh
ros2 run arm_arduino_bridge bridge
ros2 run arm_arduino_bridge direct
```
