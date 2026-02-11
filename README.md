# UGV


sudo apt update
sudo apt install ros-$ROS_DISTRO-navigation2 ros-$ROS_DISTRO-nav2-bringup ros-$ROS_DISTRO-robot-localization ros-$ROS_DISTRO-mapviz ros-$ROS_DISTRO-mapviz-plugins ros-$ROS_DISTRO-tile-map ros-$ROS_DISTRO-teleop-twist-keyboard




For PWM and PID

ros2 topic pub /pid_tune std_msgs/Float32MultiArray \
"{data: [0.9, 0.0, 0.06, 1.2, 0.0, 0.1]}"
3 angular and 3 linear

[ kp_linear, ki_linear, kd_linear,
  kp_angular, ki_angular, kd_angular ]


1.cone the repo into the src folder of workspace
then go to workspace parent dir.

  colcon build

  source install/setup.bash

  ros2 run ugv_controller pid.py

check cm_pwm for no pid velocity to pwm converter
  
  ros2 run ugv_controller cmd_pwm.py


  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
