# whi_nav2_bt_actions_server

Providing action-based controllers for mobile robots within the Nav2 framework

## SpinToPath

Calculates the heading difference between the robot and a specified point on the global path, then rotates the robot toward the target path point defined by the lookahead distance

Advertised action **SpinToPath**

```
#goal definition
nav_msgs/Path path
float64 lookahead_distance
---
#result definition
builtin_interfaces/Duration total_elapsed_time
---
#feedback definition
float32 angular_distance_traveled
```

## LocomotionOffset

This action enables the robot to move to a target pose defined by an offset relative to its current pose. Two motion strategies are supported:

1. **Direct mode**: The robot moves directly toward the target pose while maintaining the shortest path.
2. **Zigzag mode**: The robot first rotates to an appropriate orientation to increase maneuverability, then executes the translation toward the target pose.

Advertised action **LocomotionOffset**

```
#goal definition
geometry_msgs/PoseStamped offset
float64 zig_angle
float64 timeout
---
#result definition
builtin_interfaces/Duration total_elapsed_time
---
#feedback
uint8 PROC_SUCCEED=0
uint8 PROC_FAILED=1
uint8 PROC_TIMEOUT=2
uint8 PROC_ACTING=3
uint8 state
```

When `zig_angle` is zero, **Direct mode** is selected automatically. Otherwise, **Zigzag mode** is executed with the specified intermediate rotation angle (`zig_angle`).

An example of action call with CLI:

```
# up-left under zigzag mode
ros2 action send_goal /locomotion_offset whi_interfaces/action/LocomotionOffset "{offset: {header: {frame_id: 'base_link'}, pose: {position: {x: 0.2, y: 0.2, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, zig_angle: 30}"
```

> Refer to `testing_cases` for more action calls
