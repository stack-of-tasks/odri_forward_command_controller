# odri_forward_command_controller

A ROS 2 controller that simultaneously forwards **position, velocity, effort, gain\_kp and gain\_kd** commands to a set of ODRI joints through a single topic.

It follows the same design as `ros2_controllers/forward_command_controller` and reuses `std_msgs/Float64MultiArray` as the command message type.

## Wire format

The subscriber topic is `~/commands` (`std_msgs/msg/Float64MultiArray`).

For **n** joints the `data` field must contain exactly **5 × n** values, laid out as:

| Slice | Content |
|---|---|
| `data[0 .. n)` | position commands (rad) |
| `data[n .. 2n)` | velocity commands (rad/s) |
| `data[2n .. 3n)` | effort commands (N·m) |
| `data[3n .. 4n)` | gain\_kp (proportional gain) |
| `data[4n .. 5n)` | gain\_kd (derivative gain) |

A message whose `data` size differs from `5 * n_joints` is silently ignored.
Individual `NaN` values within a valid message are skipped for that interface only,
leaving the hardware value unchanged (partial updates).

## Configuration

```yaml
odri_fcc:
  ros__parameters:
    type: odri_forward_command_controller/OdriForwardCommandController
    joints:
      - joint1
      - joint2
```

## Python example

The snippet below publishes a single command to a controller named `odri_fcc`
controlling two joints.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

# Layout: [pos×n | vel×n | eff×n | kp×n | kd×n]
N_JOINTS = 2

positions  = [0.1, -0.2]   # rad
velocities = [0.0,  0.0]   # rad/s
efforts    = [0.0,  0.0]   # N·m
gains_kp   = [5.0,  5.0]
gains_kd   = [0.1,  0.1]


def main():
    rclpy.init()
    node = Node("odri_fcc_example")

    pub = node.create_publisher(
        Float64MultiArray,
        "/odri_fcc/commands",
        10,
    )

    msg = Float64MultiArray()
    msg.data = positions + velocities + efforts + gains_kp + gains_kd

    # Give the publisher time to connect before sending
    import time
    time.sleep(0.5)

    pub.publish(msg)
    node.get_logger().info(f"Published: {msg.data}")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

To publish a one-shot command directly from the terminal:

```bash
ros2 topic pub --once /odri_fcc/commands std_msgs/msg/Float64MultiArray \
  "{data: [0.1, -0.2,   0.0, 0.0,   0.0, 0.0,   5.0, 5.0,   0.1, 0.1]}"
  #        ^pos×2        ^vel×2       ^eff×2       ^kp×2        ^kd×2
```

To send a partial update (NaN skips that interface — gains unchanged here):

```bash
ros2 topic pub --once /odri_fcc/commands std_msgs/msg/Float64MultiArray \
  "{data: [0.5, -0.5,   0.0, 0.0,   1.0, -1.0,   .nan, .nan,   .nan, .nan]}"
  #        ^pos×2        ^vel×2       ^eff×2        ^kp skipped    ^kd skipped
```

## Building

```bash
cd <workspace>
colcon build --packages-select odri_forward_command_controller
source install/setup.bash
```

## Testing

```bash
colcon test --packages-select odri_forward_command_controller
colcon test-result --verbose
```
