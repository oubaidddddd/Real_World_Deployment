import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import math

class UltrasonicToLaserScan(Node):
    def __init__(self):
        super().__init__('ultrasonic_to_laserscan')

        # Topics for ultrasonic sensors
        self.sub_front = self.create_subscription(Float32, 'ultrasonic/front', self.front_callback, 10)
        self.sub_left = self.create_subscription(Float32, 'ultrasonic/left', self.left_callback, 10)
        self.sub_right = self.create_subscription(Float32, 'ultrasonic/right', self.right_callback, 10)
        self.sub_back = self.create_subscription(Float32, 'ultrasonic/back', self.back_callback, 10)

        # Publisher for LaserScan
        self.pub_laserscan = self.create_publisher(LaserScan, 'scan', 10)

        # Sensor readings
        self.front_distance = 0.0
        self.left_distance = 0.0
        self.right_distance = 0.0
        self.back_distance = 0.0

        # Timer for publishing LaserScan
        self.timer = self.create_timer(0.1, self.publish_laserscan)

    def front_callback(self, msg):
        self.front_distance = msg.data

    def left_callback(self, msg):
        self.left_distance = msg.data

    def right_callback(self, msg):
        self.right_distance = msg.data

    def back_callback(self, msg):
        self.back_distance = msg.data

    def publish_laserscan(self):
        scan = LaserScan()
        scan.header.frame_id = 'base_link'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.pi / 2
        scan.range_min = 0.02
        scan.range_max = 4.0

        # Fill ranges based on sensor data
        scan.ranges = [
            self.back_distance / 100.0,  # Convert cm to meters
            self.right_distance / 100.0,
            self.front_distance / 100.0,
            self.left_distance / 100.0
        ]

        self.pub_laserscan.publish(scan)

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicToLaserScan()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
