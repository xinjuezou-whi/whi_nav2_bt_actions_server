# whi_nav2_bt_actions_server

Providing action-based controllers for mobile robots within the Nav2 framework

## Params

```
whi_nav2_bt_actions_server:
  ros__parameters:
    use_stamped_vel: true
    cycle_frequency: 20.0
    global_frame: odom
    robot_base_frame: base_link
    transform_tolerance: 0.2
    local_costmap_topic: local_costmap/costmap_raw
    local_footprint_topic: local_costmap/published_footprint
    action_plugins: ["spin_to_path", "locomotion_offset"]
    spin_to_path:
      plugin: whi_nav2_bt_actions_server/SpinToPath
      min_rotational_vel: 0.1
      max_rotational_vel: 0.3
      rotational_acc_lim: 1.57
      simulate_ahead_time: 2.0
      check_collision: false
    locomotion_offset:
      plugin: whi_nav2_bt_actions_server/LocomotionOffset
      min_rotational_vel: 0.05
      max_rotational_vel: 0.2
      min_linear_vel: 0.05
      max_linear_vel: 0.1
      position_tolerance: 0.05
      yaw_tolerance: 5.0 # degrees
      simulate_ahead_time: 2.0
      check_collision: false
```

## SpinToPath

Calculates the heading difference between the robot and a specified point on the global path, then rotates the robot toward the target path point defined by the lookahead distance

Advertised action **SpinToPath** with name `spin_to_path`

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

Advertised action **LocomotionOffset** with name `locomotion_offset`

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

When `zig_angle` is zero, **Direct mode** is selected automatically. Otherwise, **Zigzag mode** is executed with the specified intermediate rotation angle (`zig_angle`)

Taking the y offset(0.5m) as an example to illustrate the difference of two modes:

**Direct mode**

<p>
<img width="905" height="654" alt="directly" src="https://github.com/user-attachments/assets/89be9c31-3684-4619-913d-15a8961f2d25" />
</p>

**Zigzag mode** with 45 degrees

<p>
<img width="905" height="654" alt="zigzag" src="https://github.com/user-attachments/assets/b551edfa-fd06-4b5e-b84a-1e8a2b3cb776" />
</p>

An example of an action call with CLI:

```
# up-left under zigzag mode
ros2 action send_goal /locomotion_offset whi_interfaces/action/LocomotionOffset "{offset: {header: {frame_id: 'base_link'}, pose: {position: {x: 0.2, y: 0.2, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}, zig_angle: 30}"
```

> Refer to `testing_cases` for more action call examples
