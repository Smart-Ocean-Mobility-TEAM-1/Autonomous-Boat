import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import serial

class LaserScanDriver(Node):
    def __init__(self):
        super().__init__('laser_scan_driver')

        self.arduino = serial.Serial('/dev/ttyACM1', 115200)

        self.get_logger().info("Serial connected to Arduino")

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.threshold = 0.2  # 장애물 거리 기준
        self.front_range_width = 1  # 앞쪽 검사 범위
        self.current_command = 'F'  # 초기 명령
        self.segment_size = 90;
        self.range_count = 3200;

        # 0.2초마다 명령 반복 전송
        self.timer = self.create_timer(0.5, self.send_command)

    def scan_callback(self, msg):
        right_segment = msg.ranges[0:self.segment_size]
        left_segment = msg.ranges[len(msg.ranges) - self.segment_size : len(msg.ranges)]

        # 유효한 값만 필터링
        right_valid = [r for r in right_segment if msg.range_min < r < self.threshold]
        left_valid  = [r for r in left_segment if msg.range_min < r < self.threshold]

        if len(right_valid) > len(left_valid):
            self.current_command = 'L'
        elif len(right_valid) < len(left_valid):
            self.current_command = 'R'
        elif len(right_valid) == len(left_valid) == 0:
            self.current_command = 'F'
        else:
            self.current_command = 'S'

    def send_command(self):
        self.arduino.write((self.current_command + '\n').encode())
        self.get_logger().info(f"Sent: {self.current_command}")

def main(args=None):
    rclpy.init(args=args)
    node = LaserScanDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
