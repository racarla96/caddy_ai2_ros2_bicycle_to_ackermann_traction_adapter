# caddy_ai2_ros2_bicycle_to_ackermann_traction_adapter

# Publicar un valor de posición del steering y velocidad lineal
ros2 topic pub /forward_position_command_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.3]}" -r 100
ros2 topic pub /forward_velocity_command_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.3]}" -r 100

ros2 topic pub /bicycle_steering_controller/reference geometry_msgs/msg/TwistStamped "
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: 'base_link'
twist:
  linear:
    x: 1.0
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.5
"