import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import serial 

class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.last_error = 0.0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.last_error) / dt if dt > 0 else 0.0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.last_error = error
        return output

class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('pid')
        self.pid_linear = PID(kp=1.0, ki=0.0, kd=0.0)
        self.pid_angular = PID(kp=1.0, ki=0.0, kd=0.0)
        self.neutral_pwm = 1400
        self.min_pwm = 1000
        self.max_pwm = 2000

        self.serial_port = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        self.get_logger().info('Serial port opened')
        self.desired_twist = Twist()
        self.current_linear_vel = 0.0
        self.last_time = self.get_clock().now()

        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )

    def cmd_vel_callback(self, msg: Twist):
        self.desired_twist = msg

    def imu_callback(self, msg: Imu):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        if dt <= 0:
            self.last_time = current_time
            return

        accel_x = msg.linear_acceleration.x 
        self.current_linear_vel += accel_x * dt

        current_angular_vel = msg.angular_velocity.z
        error_linear = self.desired_twist.linear.x - self.current_linear_vel
        error_angular = self.desired_twist.angular.z - current_angular_vel

        output_linear = self.pid_linear.compute(error_linear, dt)
        output_angular = self.pid_angular.compute(error_angular, dt)

        left_pwm = self.neutral_pwm + output_linear - output_angular
        right_pwm = self.neutral_pwm + output_linear + output_angular

        left_pwm = max(self.min_pwm, min(self.max_pwm, left_pwm))
        right_pwm = max(self.min_pwm, min(self.max_pwm, right_pwm))

        # Send over serial (format: L<pwm>,R<pwm>\n)
        command = f"L{int(left_pwm)},R{int(right_pwm)}\n"
        self.serial_port.write(command.encode())
        self.get_logger().info(f'Sent: {command.strip()}')

        self.last_time = current_time

    def destroy_node(self):
        self.serial_port.close()
        self.get_logger().info('Serial port closed')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()