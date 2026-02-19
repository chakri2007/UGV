#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
import serial
import math

class NMEAToFix(Node):

    def __init__(self):
        super().__init__('gps')

        # -------- PARAMETERS --------
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value

        # -------- SERIAL --------
        self.ser = serial.Serial(port, baud, timeout=1.0)
        self.get_logger().info(f"Opened serial port {port} at {baud}")

        # -------- PUBLISHER --------
        self.fix_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)

        self.timer = self.create_timer(0.05, self.read_serial)

        # temp storage
        self.latitude = None
        self.longitude = None
        self.altitude = 0.0
        self.fix_valid = False

    # -------------------------------------------------
    def read_serial(self):
        try:
            line = self.ser.readline().decode('ascii', errors='ignore').strip()
            if not line.startswith('$'):
                return

            if 'GGA' in line:
                self.parse_gga(line)
            elif 'RMC' in line:
                self.parse_rmc(line)

            if self.fix_valid and self.latitude is not None:
                self.publish_fix()

        except Exception as e:
            self.get_logger().warn(str(e))

    # -------------------------------------------------
    def parse_gga(self, sentence):
        parts = sentence.split(',')

        # Fix quality
        fix_quality = int(parts[6]) if parts[6].isdigit() else 0
        self.fix_valid = fix_quality > 0

        if not self.fix_valid:
            return

        self.latitude = self.convert_to_decimal(parts[2], parts[3])
        self.longitude = self.convert_to_decimal(parts[4], parts[5])

        try:
            self.altitude = float(parts[9])
        except:
            self.altitude = 0.0

    # -------------------------------------------------
    def parse_rmc(self, sentence):
        parts = sentence.split(',')

        # Status A = valid, V = invalid
        if parts[2] != 'A':
            self.fix_valid = False

    # -------------------------------------------------
    def convert_to_decimal(self, raw, direction):
        if raw == '':
            return None

        degrees = int(float(raw) / 100)
        minutes = float(raw) - degrees * 100
        decimal = degrees + minutes / 60.0

        if direction in ['S', 'W']:
            decimal *= -1

        return decimal

    # -------------------------------------------------
    def publish_fix(self):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps_link'

        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS

        msg.latitude = self.latitude
        msg.longitude = self.longitude
        msg.altitude = self.altitude

        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        self.fix_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = NMEAToFix()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
