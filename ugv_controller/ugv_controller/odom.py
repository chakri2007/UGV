import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
import math
from rclpy.time import Time

class CmdVelToOdomNode(Node):
    def __init__(self):
        super().__init__('odom')
        
        # Parameters (tune as needed)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_footprint')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('use_imu_gyro', True)  # Use IMU for angular velocity
        
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.use_imu_gyro = self.get_parameter('use_imu_gyro').value
        
        # State variables
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0  # Yaw
        self.vx = 0.0  # Linear x velocity
        self.vth = 0.0  # Angular z velocity (from cmd_vel or imu)
        
        self.last_time = self.get_clock().now()
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        
        # Publisher
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        
        # Timer for integration (50 Hz)
        self.timer = self.create_timer(0.02, self.update_odom)
        
        self.get_logger().info('CmdVelToOdomNode started')

    def cmd_vel_callback(self, msg):
        self.vx = msg.linear.x
        if not self.use_imu_gyro:
            self.vth = msg.angular.z

    def imu_callback(self, msg):
        if self.use_imu_gyro:
            self.vth = msg.angular_velocity.z  # Use gyro z for better yaw rate

    def update_odom(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        # Integrate velocities (simple Euler integration)
        delta_x = self.vx * math.cos(self.th) * dt
        delta_y = self.vx * math.sin(self.th) * dt
        delta_th = self.vth * dt
        
        self.x += delta_x
        self.y += delta_y
        self.th += delta_th
        
        # Quaternion from yaw
        quat = Quaternion()
        quat.z = math.sin(self.th / 2.0)
        quat.w = math.cos(self.th / 2.0)
        
        # Publish Odometry
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = quat
        
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.vth
        
        # Covariances (tune these; higher values mean less trust in this odom)
        odom.pose.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,  # X
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,   # Y
            0.0, 0.0, 1000.0, 0.0, 0.0, 0.0, # Z (ignored in 2D)
            0.0, 0.0, 0.0, 1000.0, 0.0, 0.0, # Roll
            0.0, 0.0, 0.0, 0.0, 1000.0, 0.0, # Pitch
            0.0, 0.0, 0.0, 0.0, 0.0, 0.05    # Yaw
        ]
        odom.twist.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,  # VX
            0.0, 1000.0, 0.0, 0.0, 0.0, 0.0, # VY (ignored)
            0.0, 0.0, 1000.0, 0.0, 0.0, 0.0, # VZ
            0.0, 0.0, 0.0, 1000.0, 0.0, 0.0, # VRoll
            0.0, 0.0, 0.0, 0.0, 1000.0, 0.0, # VPitch
            0.0, 0.0, 0.0, 0.0, 0.0, 0.05    # VYaw
        ]
        
        self.odom_pub.publish(odom)
        
        # Publish TF
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = self.frame_id
            t.child_frame_id = self.child_frame_id
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation = quat
            self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToOdomNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()