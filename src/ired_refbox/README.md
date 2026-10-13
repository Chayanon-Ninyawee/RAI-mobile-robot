# ired_refbox

ROS 2 communication node for the RoboCup Logistics League RefBox.

## Responsibilities

- Receive and publish RefBox protobuf messages.
- Send periodic robot BeaconSignal messages.
- Use the robot's ROS pose when available.
- Keep RefBox communication independent of Nav2.

## Dependencies

- ROS 2 Jazzy
- protobuf_comm
- Generated RCLL protobuf messages
- Protobuf
- Boost
- OpenSSL

## Build

The default external dependency location is:

```
~/rcll/rcll_refbox_comm_example
```

To use another location:

```
colcon build --packages-select ired_refbox \
  --cmake-args \
  -DREFBOX_COMM_ROOT=/path/to/rcll_refbox_comm_example
```

## Run

```
ros2 launch ired_refbox refbox.launch.py
```

## Topics

- `/refbox/messages`
- `/refbox/game_state`
- `/refbox/navigation_routes`
- `/refbox/machine_report_info`

The node subscribes to `/amcl_pose` by default and publishes periodic
BeaconSignal messages to the RefBox private channel.

Machine reports are not sent automatically.
