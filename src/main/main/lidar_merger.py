import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import message_filters
import tf2_ros
from tf2_ros import TransformException
import math

class LidarMerger(Node): 
    def __init__(self):
        super().__init__('lidar_merger')

        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("scan_output", "scan_merged")
        self.declare_parameter("min_range", 0.1)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.lidar1_sub = message_filters.Subscriber(self, LaserScan, "scan_input1")
        self.lidar2_sub = message_filters.Subscriber(self, LaserScan, "scan_input2")

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.lidar1_sub, self.lidar2_sub], queue_size=10, slop=0.1
        )
        self.ts.registerCallback(self.merge_scans)

        self.pub = self.create_publisher(LaserScan, self.get_parameter("scan_output").value, 10)

    # Inerpolate between can points to match the target resolution
    def resample_scan(self, range, scan_msg, target_increment):
        ranges = np.array(range, dtype=np.float32)
        if ranges.size == 0:
            return []

        if scan_msg.angle_increment <= 0.0 or target_increment <= 0.0:
            return ranges.tolist()

        original_angles = scan_msg.angle_min + np.arange(ranges.size, dtype=np.float32) * scan_msg.angle_increment
        target_angles = np.arange(
            scan_msg.angle_min,
            scan_msg.angle_max,
            target_increment
        )

        out_of_range = scan_msg.range_max + 1.0
        ranges[np.isinf(ranges)] = out_of_range
        ranges[np.isnan(ranges)] = out_of_range

        resampled_ranges = np.interp(
            target_angles,
            original_angles,
            ranges,
            left=scan_msg.range_max + 1.0,
            right=scan_msg.range_max + 1.0
        )

        resampled_ranges[resampled_ranges > scan_msg.range_max] = float('inf')
                      
        return resampled_ranges.tolist()

    # Filter out points that are below the minimum range threshold
    def apply_min_range(self, ranges):
        min_range = self.get_parameter("min_range").value
        return [float("inf") if value < min_range else value for value in ranges]
    
    # Transform scan points to target frame
    def transform_scan(self, ranges, scan_msgs):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.get_parameter("target_frame").value,
                scan_msgs.header.frame_id,
                scan_msgs.header.stamp,
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
        except TransformException as ex:
            self.get_logger().warn(f"Could not transform {scan_msgs.header.frame_id} to {self.get_parameter('target_frame').value}: {ex}")
            return ranges
        
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        q = transform.transform.rotation

        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        points = []
        for i, r in enumerate(ranges):
            if not math.isfinite(r) or r < scan_msgs.range_min or r > scan_msgs.range_max:
                points.append(None)
                continue

            angle = scan_msgs.angle_min + i * scan_msgs.angle_increment

            lx = r * math.cos(angle)
            ly = r * math.sin(angle)

            bx = tx + lx * math.cos(yaw) - ly * math.sin(yaw)
            by = ty + lx * math.sin(yaw) + ly * math.cos(yaw)

            points.append((bx, by))

        return points

    # Convert transformed points back to ranges for the merged scan
    def points_to_ranges(self, points, scan_msg):
        num_readings = int((scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment)
        ranges = [float('inf')] * num_readings

        for point in points:
            if point is None:
                continue

            bx, by = point

            angle = math.atan2(by, bx)
            idx = int(round((angle - scan_msg.angle_min) / scan_msg.angle_increment))

            if idx < 0 or idx >= num_readings:
                continue

            r = math.sqrt(bx * bx + by * by)
            if r < scan_msg.range_min or r > scan_msg.range_max:
                continue

            if r < ranges[idx]:
                ranges[idx] = r

        return ranges
    
    def merge_scans(self, scan1_msg, scan2_msg):
        scan1_stamp_ns = scan1_msg.header.stamp.sec * 1_000_000_000 + scan1_msg.header.stamp.nanosec
        scan2_stamp_ns = scan2_msg.header.stamp.sec * 1_000_000_000 + scan2_msg.header.stamp.nanosec
        merged_stamp_ns = (scan1_stamp_ns + scan2_stamp_ns) // 2

        merged_scan = LaserScan()
        merged_scan.header.stamp.sec = int(merged_stamp_ns // 1_000_000_000)
        merged_scan.header.stamp.nanosec = int(merged_stamp_ns % 1_000_000_000)
        merged_scan.header.frame_id = self.get_parameter("target_frame").value
        merged_scan.angle_min = -math.pi
        merged_scan.angle_max = math.pi
        merged_scan.scan_time = max(scan1_msg.scan_time, scan2_msg.scan_time)
        merged_scan.range_min = min(scan1_msg.range_min, scan2_msg.range_min)
        merged_scan.range_max = max(scan1_msg.range_max, scan2_msg.range_max)

        if (scan1_msg.angle_increment < scan2_msg.angle_increment):
            merged_scan.angle_increment = scan1_msg.angle_increment
            merged_scan.time_increment = scan1_msg.time_increment
        else:
            merged_scan.angle_increment = scan2_msg.angle_increment
            merged_scan.time_increment = scan2_msg.time_increment

        resampled_ranges1 = self.resample_scan(scan1_msg.ranges, scan1_msg, merged_scan.angle_increment)
        resampled_ranges2 = self.resample_scan(scan2_msg.ranges, scan2_msg, merged_scan.angle_increment)

        filtered_ranges1 = self.apply_min_range(resampled_ranges1)
        filtered_ranges2 = self.apply_min_range(resampled_ranges2)

        transformed_ranges1 = self.transform_scan(filtered_ranges1, scan1_msg)
        transformed_ranges2 = self.transform_scan(filtered_ranges2, scan2_msg)

        all_points = transformed_ranges1 + transformed_ranges2
        merged_scan.ranges = self.points_to_ranges(all_points, merged_scan)

        self.pub.publish(merged_scan)

def main(args=None):
    rclpy.init(args=args)
    lidar_merger = LidarMerger()
    try:
        rclpy.spin(lidar_merger)
    finally:
        lidar_merger.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()