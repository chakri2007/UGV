#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray
import time


class CmdVelToPWMSequencer(Node):

    def __init__(self):
        super().__init__('cmd_vel_to_pwm_sequencer')

        self.NEUTRAL = 1477

        # throttle
        self.THROTTLE_MIN = 1032   # forward
        self.THROTTLE_MAX = 1915   # reverse

        # steering
        self.STEER_LEFT = 1032
        self.STEER_RIGHT = 1915

        # steering deadband
        self.DEAD_MIN = 1400
        self.DEAD_MAX = 1600

        # motion filtering
        self.deadzone = 0.05
        self.angular_filter = 0.15

        # sequencing timings (TUNE ON ROBOT)
        self.steer_duration = 0.3
        self.drive_duration = 0.4

        self.current_cmd = Twist()
        self.mode = "stop"
        self.phase_start = time.time()

        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        self.pub = self.create_publisher(
            Int16MultiArray,
            '/pwm_signals',
            10
        )

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("CmdVel → PWM Sequencer Started")

    def map_linear(self, linear):

        if linear > 0:
            return int(
                self.NEUTRAL -
                linear * (self.NEUTRAL - self.THROTTLE_MIN)
            )
        else:
            return int(
                self.NEUTRAL +
                abs(linear) * (self.THROTTLE_MAX - self.NEUTRAL)
            )

    def map_angular(self, angular):

        if abs(angular) < self.deadzone:
            return self.NEUTRAL

        # RIGHT TURN
        if angular > 0:
            return int(
                self.DEAD_MAX +
                angular * (self.STEER_RIGHT - self.DEAD_MAX)
            )

        # LEFT TURN
        else:
            return int(
                self.DEAD_MIN -
                abs(angular) * (self.DEAD_MIN - self.STEER_LEFT)
            )

    def cmd_callback(self, msg):
        self.current_cmd = msg

    def control_loop(self):

        linear = max(min(self.current_cmd.linear.x, 1.0), -1.0)
        angular = max(min(self.current_cmd.angular.z, 1.0), -1.0)

        if abs(angular) < self.angular_filter:
            angular = 0.0

        now = time.time()

        throttle_pwm = self.NEUTRAL
        steering_pwm = self.NEUTRAL

        if abs(linear) < self.deadzone and abs(angular) < self.deadzone:
            self.mode = "stop"

        elif abs(linear) > self.deadzone and abs(angular) > self.deadzone:

            if self.mode not in ["steer", "drive"]:
                self.mode = "steer"
                self.phase_start = now

            if self.mode == "steer":
                throttle_pwm = self.NEUTRAL
                steering_pwm = self.map_angular(angular)

                if now - self.phase_start > self.steer_duration:
                    self.mode = "drive"
                    self.phase_start = now

            elif self.mode == "drive":
                throttle_pwm = self.map_linear(linear)
                steering_pwm = self.NEUTRAL

                if now - self.phase_start > self.drive_duration:
                    self.mode = "steer"
                    self.phase_start = now

        # -----------------------------
        # STRAIGHT DRIVE
        # -----------------------------
        elif abs(linear) > self.deadzone:
            self.mode = "drive"
            throttle_pwm = self.map_linear(linear)
            steering_pwm = self.NEUTRAL

        # -----------------------------
        # PURE TURN
        # -----------------------------
        elif abs(angular) > self.deadzone:
            self.mode = "steer"
            throttle_pwm = self.NEUTRAL
            steering_pwm = self.map_angular(angular)

        # -----------------------------
        # PUBLISH PWM
        # -----------------------------
        pwm_msg = Int16MultiArray()
        pwm_msg.data = [throttle_pwm, steering_pwm]

        self.pub.publish(pwm_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToPWMSequencer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
