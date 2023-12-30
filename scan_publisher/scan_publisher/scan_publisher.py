import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LaserPublisher(Node):
    def __init__(self, name):
        super().__init__(name)
        self.publisher_ = self.create_publisher(LaserScan, 'laser_scan', 10)
        self.subsriber = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)

    def scan_callback(self, msg):
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    laser_publisher = LaserPublisher('laser_publisher')
    rclpy.spin(laser_publisher)
    laser_publisher.destroy_node()
    rclpy.shutdown()