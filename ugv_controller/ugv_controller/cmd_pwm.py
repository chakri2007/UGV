import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class UGVControlNode(Node):
    def __init__(self):
        super().__init__('cmd_pwm')

        # Declare parameters with defaults
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('max_linear_speed', 1.0)  # m/s
        self.declare_parameter('max_angular_speed', 1.0)  # rad/s

        # Get parameters
        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value

        # PWM constants
        self.STEERING_MIN = 1032  # Left max
        self.STEERING_MAX = 1915  # Right max
        self.STEERING_NEUTRAL = 1477

        self.THROTTLE_MIN = 1032  # Forward max
        self.THROTTLE_MAX = 1915  # Backward max
        self.THROTTLE_NEUTRAL = 1477

        # Initialize serial connection
        try:
            self.ser = serial.Serial(serial_port, baud_rate, timeout=1)
            self.get_logger().info(f'Serial port opened: {serial_port} at {baud_rate} baud')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            rclpy.shutdown()

        # Subscriber to cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

    def cmd_vel_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        # Compute throttle PWM
        if linear > 0:
            ratio = min(linear / self.max_linear, 1.0)
            throttle_pwm = self.THROTTLE_NEUTRAL - (self.THROTTLE_NEUTRAL - self.THROTTLE_MIN) * ratio
        elif linear < 0:
            ratio = min(-linear / self.max_linear, 1.0)
            throttle_pwm = self.THROTTLE_NEUTRAL + (self.THROTTLE_MAX - self.THROTTLE_NEUTRAL) * ratio
        else:
            throttle_pwm = self.THROTTLE_NEUTRAL

        # Clamp throttle
        throttle_pwm = max(self.THROTTLE_MIN, min(self.THROTTLE_MAX, throttle_pwm))

        # Compute steering PWM
        if angular > 0:  # Left turn
            ratio = min(angular / self.max_angular, 1.0)
            steering_pwm = self.STEERING_NEUTRAL - (self.STEERING_NEUTRAL - self.STEERING_MIN) * ratio
        elif angular < 0:  # Right turn
            ratio = min(-angular / self.max_angular, 1.0)
            steering_pwm = self.STEERING_NEUTRAL + (self.STEERING_MAX - self.STEERING_NEUTRAL) * ratio
        else:
            steering_pwm = self.STEERING_NEUTRAL

        # Clamp steering
        steering_pwm = max(self.STEERING_MIN, min(self.STEERING_MAX, steering_pwm))

        # Send over serial (format: steering,throttle\n)
        command = f"{int(steering_pwm)},{int(throttle_pwm)}\n"
        self.ser.write(command.encode())
        self.get_logger().info(f'Sent: {command.strip()}')

def main(args=None):
    rclpy.init(args=args)
    node = UGVControlNode()
    rclpy.spin(node)
    node.ser.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()