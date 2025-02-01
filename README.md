# Kanga Code Implementation (beta)

ROS2 will auto source from bashrc

## For cameras
```
cd cameras
. install/setup.bash
ros2 run multi_camera_node multi_camera_node
```

## For Science
### Control from BaseStation
```
cd telecoms
. install/setup.bash
ros2 run science_control science_control
```

### Feedback to BaseStation
```
cd telecoms
. install/setup.bash
ros2 run science_control science_feedback
```

## For Drivetrain Control
Note: Control of motors not yet implemented
### Control from BaseStation
```
cd telecoms
. install/setup.bash
ros2 run drivetrain_control motor_control
```

### Feedback to BaseStation
```
cd telecoms
. install/setup.bash
ros2 run drivetrain_control motor_feedback
```