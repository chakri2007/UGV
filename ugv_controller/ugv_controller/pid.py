#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import serial
import time


# ---------------- PID CLASS (UNCHANGED) ----------------
class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0.0
        self.integral = 0.0

    def update_gains(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative


# ---------------- MAIN NODE ----------------
class CmdVelPIDSequencer(Node):

    def __init__(self):
        super().__init__('cmd_vel_pid_sequencer')

        # PWM CONSTANTS
        self.NEUTRAL = 1477
        self.THROTTLE_MIN = 1032
        self.THROTTLE_MAX = 1915
        self.STEER_LEFT = 1032
        self.STEER_RIGHT = 1915
        self.DEAD_MIN = 1400
        self.DEAD_MAX = 1600

        self.deadzone = 0.05
        self.angular_filter = 0.15

        self.steer_duration = 0.3
        self.drive_duration = 0.4

        # PID CONTROLLERS (UNCHANGED)
        self.pid_linear = PID(0.8, 0.0, 0.05)
        self.pid_angular = PID(1.0, 0.0, 0.08)

        self.current_cmd = Twist()

        self.meas_linear = 0.0
        self.meas_angular = 0.0

        self.mode = "stop"
        self.phase_start = time.time()
        self.last_time = time.time()

        # SERIAL
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.01)

        # SUBSCRIBERS
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        self.create_subscription(
            Float32MultiArray,
            '/pid_tune',
            self.pid_tune_callback,
            10
        )

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("PID Sequencer with Serial Output Started")


    # ---------------- CALLBACKS ----------------
    def cmd_callback(self, msg):
        self.current_cmd = msg


    def pid_tune_callback(self, msg):

        if len(msg.data) != 6:
            self.get_logger().warn("pid_tune requires 6 values")
            return

        kp_l, ki_l, kd_l, kp_a, ki_a, kd_a = msg.data

        self.pid_linear.update_gains(kp_l, ki_l, kd_l)
        self.pid_angular.update_gains(kp_a, ki_a, kd_a)

        self.get_logger().info(
            f"Updated PID | Linear: {kp_l},{ki_l},{kd_l} | "
            f"Angular: {kp_a},{ki_a},{kd_a}"
        )


    # ---------------- SERIAL FEEDBACK ----------------
    def read_feedback(self):
        try:
            line = self.ser.readline().decode().strip()
            if line:
                v, w = line.split(',')
                self.meas_linear = float(v)
                self.meas_angular = float(w)
        except:
            pass


    # ---------------- MAPPING FUNCTIONS ----------------
    def map_linear(self, linear):
        if linear > 0:
            return int(self.NEUTRAL -
                       linear * (self.NEUTRAL - self.THROTTLE_MIN))
        else:
            return int(self.NEUTRAL +
                       abs(linear) * (self.THROTTLE_MAX - self.NEUTRAL))


    def map_angular(self, angular):
        if abs(angular) < self.deadzone:
            return self.NEUTRAL

        if angular > 0:
            return int(self.DEAD_MAX +
                       angular * (self.STEER_RIGHT - self.DEAD_MAX))
        else:
            return int(self.DEAD_MIN -
                       abs(angular) * (self.DEAD_MIN - self.STEER_LEFT))


    # ---------------- CONTROL LOOP ----------------
    def control_loop(self):

        self.read_feedback()

        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        cmd_linear = self.current_cmd.linear.x
        cmd_angular = self.current_cmd.angular.z

        if abs(cmd_angular) < self.angular_filter:
            cmd_angular = 0.0

        # ---------- PID (UNCHANGED) ----------
        error_lin = cmd_linear - self.meas_linear
        error_ang = cmd_angular - self.meas_angular
        self.get_logger().debug(f"Errors | Linear: {error_lin:.3f} | Angular: {error_ang:.3f}")

        corr_lin = self.pid_linear.compute(error_lin, dt)
        corr_ang = self.pid_angular.compute(error_ang, dt)

        linear = max(min(cmd_linear + corr_lin, 1.0), -1.0)
        angular = max(min(cmd_angular + corr_ang, 1.0), -1.0)
        # ------------------------------------

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

        elif abs(linear) > self.deadzone:
            self.mode = "drive"
            throttle_pwm = self.map_linear(linear)
            steering_pwm = self.NEUTRAL

        elif abs(angular) > self.deadzone:
            self.mode = "steer"
            throttle_pwm = self.NEUTRAL
            steering_pwm = self.map_angular(angular)

        # -------- OUTPUT FORMAT (MATCHING YOUR REFERENCE) --------
        cmd_str = f"{steering_pwm},{throttle_pwm}\n"
        self.get_logger().info(f"Publishing PWM: {cmd_str.strip()}")
        self.ser.write(cmd_str.encode('ascii'))


# ---------------- MAIN ----------------
def main(args=None):
    rclpy.init(args=args)
    node = CmdVelPIDSequencer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
