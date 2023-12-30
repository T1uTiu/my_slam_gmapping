from collections import defaultdict
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

max_scan_cnt = 1500

class FeaturePublisher(Node):
    def __init__(self, name):
        super().__init__(name)
        self.publisher_ = self.create_publisher(LaserScan, 'feature_scan', 10)
        self.subsriber = self.create_subscription(LaserScan, 'laser_scan', self.scan_callback, 10)
        self.edge_threshold = 0.1

    def scan_callback(self, msg: LaserScan):
        scan_smoothness, scan_curvature = [[0.0,0.0]]*max_scan_cnt, [0]*max_scan_cnt # curvature&index, curvature
        map_index = defaultdict(int) # valid index in origin 
        cnt = 0
        new_scan = [0]*max_scan_cnt

        # remove invalid data
        for i in range(len(msg.ranges)):
            if msg.ranges[i] == float('Inf') or msg.ranges[i] == float('nan'):
                continue
            # valid data's index in origin is i, in new_scan is cnt
            map_index[cnt] = i 
            new_scan[cnt] = msg.ranges[i]
            cnt += 1
        
        # compute curvature in valid datas
        for i in range(5, cnt-5):
            diff_range = new_scan[i-5]+ new_scan[i-4]+ new_scan[i-3]+ new_scan[i-2]+ new_scan[i-1] - new_scan[i]*10 + new_scan[i+1]+ new_scan[i+2]+ new_scan[i+3]+ new_scan[i+4]+ new_scan[i+5]
            scan_curvature[i] = diff_range*diff_range
            scan_smoothness[i] = [scan_curvature[i], i]

        # msg
            corner_msg = LaserScan()
            corner_msg.header = msg.header
            corner_msg.angle_min = msg.angle_min
            corner_msg.angle_max = msg.angle_max
            corner_msg.angle_increment = msg.angle_increment
            corner_msg.range_min = msg.range_min
            corner_msg.range_max = msg.range_max

            corner_msg.ranges = [0.0]*max_scan_cnt
        
        # feature detection
        for j in range(6):
            # divide the valid data into 6 parts
            start_index = int(cnt*j/6)
            end_index = int(cnt*(j+1)/6)-1
            if end_index >= cnt:
                end_index = cnt-1

            sorted_scan = sorted(scan_smoothness[start_index:end_index+1], key=lambda x:x[0])
            largest_picked_num = 0
            for k in range(end_index, start_index-1, -1):
                index = sorted_scan[k-start_index][1]
                if scan_smoothness[k][0] > self.edge_threshold:
                    largest_picked_num += 1
                    if largest_picked_num <= 20:
                        corner_msg.ranges[map_index[index]] = msg.ranges[map_index[index]]
                    else:
                        break
        
        self.publisher_.publish(corner_msg)

def main(args=None):
    rclpy.init(args=args)
    feature_publisher = FeaturePublisher('feature_publisher')
    rclpy.spin(feature_publisher)
    feature_publisher.destroy_node()
    rclpy.shutdown()