# bicycle_to_ackermann_traction_adapter

A `ros2_control` chainable controller that converts a **bicycle-model center traction velocity** into individual **left and right wheel velocities**, accounting for the differential effect when the vehicle turns.

When an Ackermann vehicle steers, the outer wheel travels a longer arc than the inner wheel in the same time. If both driven wheels were commanded the same angular velocity, the vehicle would slip. This controller computes the correct per-wheel velocity from the center velocity and the current steering angle every control cycle.

## Kinematics

Given the center traction angular velocity **ω_c** (rad/s), the current center steering angle **δ**, and the vehicle geometry:

```
v = ω_c · wheel_radius

transmission_factor = (track_width · tan(δ)) / (2 · wheelbase)

ω_left  = v · (1 − transmission_factor) / wheel_radius
ω_right = v · (1 + transmission_factor) / wheel_radius
```

State feedback path (for upstream odometry): the controller exposes the average linear velocity `((ω_right + ω_left) / 2) · wheel_radius` as a chained state interface.

## Controller chain

```
[bicycle_steering_controller]
        │ reference interface: center_traction_joint/velocity (rad/s)
        ▼
[bicycle_to_ackermann_traction_adapter]   ← this package
        │ reads state: <steering_adapter>/center_steering_joint/position
        │ command interfaces: rear_right_wheel_joint/velocity (rad/s)
        │                     rear_left_wheel_joint/velocity  (rad/s)
        ▼
[hardware / ros2_control joints]
```

> The steering angle is read from the state interface exported by
> `bicycle_to_ackermann_steering_adapter` via `input_steering_name`.

## Parameters

| Parameter | Type | Description |
|---|---|---|
| `wheelbase` | `double` (> 0) | Distance between front and rear axles (m). See [Wikipedia: Wheelbase](https://en.wikipedia.org/wiki/Wheelbase). |
| `track_width` | `double` (> 0) | Distance between the two rear driven wheels (m). |
| `wheel_radius` | `double` (> 0) | Radius of the driven wheels (m). |
| `input_steering_name` | `string` | Full name of the steering state interface to read (e.g. `bicycle_to_ackermann_steering_adapter/center_steering_joint`). |
| `input_traction_name` | `string` | Name of the chained traction reference interface (e.g. `center_traction_joint`). |
| `output_traction_names` | `string[]` (size = 2) | Hardware joint names in order: `[right_joint, left_joint]`. |

### Example configuration

```yaml
bicycle_to_ackermann_traction_adapter:
  ros__parameters:
    type: 'bicycle_to_ackermann_traction_adapter/BicycleToAckermannTractionAdapter'
    wheelbase: 1.7
    track_width: 1.0
    wheel_radius: 0.3
    input_steering_name: 'bicycle_to_ackermann_steering_adapter/center_steering_joint'
    input_traction_name: 'center_traction_joint'
    output_traction_names: ['rear_right_wheel_joint', 'rear_left_wheel_joint']
```

## Build

```bash
cd <ros2_ws>
colcon build --packages-select bicycle_to_ackermann_traction_adapter
source install/setup.bash
```
