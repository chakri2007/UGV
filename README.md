# UGV

This is the git repo for the **IROC-u** Unmanned Ground Vehicle (UGV). It contains the ROS 2 control stack for a differential/skid-steer-style rover driven over serial PWM, along with a standalone GPS waypoint generator for area-coverage missions.

---

## Repository Layout

```
UGV/
├── ugv_controller/              # ROS 2 package — control, sensing, actuation
│   ├── ugv_controller/
│   │   ├── pid.py                # cmd_vel PID + IMU feedback → PWM over serial
│   │   ├── cmd_pwm.py             # cmd_vel → PWM sequencer (no IMU feedback)
│   │   ├── gps.py                 # NMEA serial GPS → /gps/fix (NavSatFix)
│   │   └── odom.py                # cmd_vel integration → /odom + TF
│   └── launch/
│       └── ugv_sensors.launch.py  # brings up pid, cmd_pwm, gps, odom together
│
└── surveillance/
    └── path_plan.py              # KML polygon → lawnmower-pattern GPS waypoints (JSON)
```

## How It Fits Together

```
KML area file
     │  (path_plan.py)
     ▼
waypoints.json  ──►  [your waypoint follower / mission node]
                              │
                              ▼  geometry_msgs/Twist on /cmd_vel
                    ┌─────────────────────┐
                    │ pid.py  OR  cmd_pwm.py│
                    └─────────────────────┘
                              │  PWM string over serial (USB)
                              ▼
                      Arduino / motor controller
```

- **`gps.py`** reads raw NMEA sentences (`GGA`/`RMC`) off a serial GPS module and publishes `sensor_msgs/NavSatFix` on `/gps/fix`.
- **`odom.py`** integrates `/cmd_vel` (optionally fused with `/imu` for yaw rate) into a 2D odometry estimate, publishing `nav_msgs/Odometry` on `/odom` and broadcasting the `odom → base_footprint` TF.
- **`cmd_pwm.py`** is the simplest path from `/cmd_vel` to motors: it maps linear/angular velocity directly to throttle/steering PWM values and writes them as a `"steering,throttle\n"` string to the Arduino over USB serial, with a steer-then-drive sequencing scheme and a watchdog that sends neutral PWM if `/cmd_vel` goes stale.
- **`pid.py`** is the closed-loop version: it reads IMU-derived feedback back from a second serial port, runs PID correction on linear/angular velocity error before mapping to PWM, and exposes live gain tuning via `/pid_tune`.
- **`surveillance/path_plan.py`** is a standalone (non-ROS) script: it parses a polygon from a KML file and generates a back-and-forth "lawnmower" coverage path as a list of GPS waypoints, written out to `waypoints.json`. This is meant to be fed into whatever waypoint-following/mission node drives `/cmd_vel` toward each point in sequence.

> Note: `pid.py` and `cmd_pwm.py` are two different control strategies for the same actuation path (with vs. without IMU feedback) — run one or the other, not both, unless you've remapped topics/ports to avoid clashing on the same serial device.

---

## Quick Start

### Prerequisites
```bash
sudo apt update
sudo apt install ros-$ROS_DISTRO-navigation2 ros-$ROS_DISTRO-nav2-bringup \
  ros-$ROS_DISTRO-robot-localization ros-$ROS_DISTRO-mapviz \
  ros-$ROS_DISTRO-mapviz-plugins ros-$ROS_DISTRO-tile-map \
  ros-$ROS_DISTRO-teleop-twist-keyboard
pip install pyserial
```

### Build
```bash
# clone this repo into the src/ folder of your workspace, then from the workspace root:
colcon build
source install/setup.bash
```

### Run
```bash
# Bring up GPS, odometry, and one of the two control paths
ros2 launch ugv_controller ugv_sensors.launch.py

# Or run nodes individually, e.g. the PID path with IMU feedback:
ros2 run ugv_controller pid

# ...or the simpler direct PWM path:
ros2 run ugv_controller cmd_pwm

# Drive manually for testing:
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Tune PID gains live (kp_linear, ki_linear, kd_linear, kp_angular, ki_angular, kd_angular):
ros2 topic pub /pid_tune std_msgs/Float32MultiArray "{data: [0.9, 0.0, 0.06, 1.2, 0.0, 0.1]}"

# GPS node with custom port/baud:
ros2 run ugv_controller gps --ros-args -p port:=/dev/ttyUSB0 -p baud:=115200
```

### Generate coverage waypoints
```bash
cd surveillance
python3 path_plan.py   # edit input_file / output_file / spacing at the bottom of the script
```
This reads an `area.kml` polygon and writes `waypoints.json` containing the lawnmower-pattern GPS points to follow.

---

## Hardware Notes

- Motor control is via a serial-connected Arduino (or similar) expecting PWM values as a comma-separated string (`"<steering_pwm>,<throttle_pwm>\n"`), typically on `/dev/ttyACM0`.
- `pid.py` expects a **second** serial connection (`/dev/ttyACM1`) streaming IMU-derived orientation feedback (`qx,qy,qz,qw`) back from the same microcontroller setup.
- GPS defaults to `/dev/ttyUSB0` at 115200 baud — override with the `port`/`baud` ROS parameters if different.
- Double-check serial device names (`ttyACM0`/`ttyACM1`/`ttyUSB0`) against your actual setup; these can shift depending on USB enumeration order.
