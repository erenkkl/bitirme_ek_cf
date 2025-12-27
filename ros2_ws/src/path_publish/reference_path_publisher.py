#!/usr/bin/env python3
import os
import csv
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class ReferencePathPublisher(Node):
    def __init__(self):
        super().__init__('reference_path_publisher')

        self.declare_parameter('reference_csv_path', '/home/eren/Bitirme/bitirme_dataset/references.csv')
        self.declare_parameter('output_topic', '/trajectory/reference')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_hz', 1.0)

        # Unity Y-up -> ROS Z-up (x,y,z)_unity -> (x,z,y)_ros
        self.declare_parameter('swap_unity_to_ros', True)

        # 2D görünüm: z=0 (RViz’de okunaklı)
        self.declare_parameter('use_2d', True)

        csv_path = self.get_parameter('reference_csv_path').value
        self.out_topic = self.get_parameter('output_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        hz = float(self.get_parameter('publish_hz').value)
        self.swap_axes = bool(self.get_parameter('swap_unity_to_ros').value)
        self.use_2d = bool(self.get_parameter('use_2d').value)

        if not csv_path or not os.path.exists(csv_path):
            raise FileNotFoundError(f"reference_csv_path not found: {csv_path}")

        self.path_msg = self._build_path_from_csv(csv_path)

        self.pub = self.create_publisher(Path, self.out_topic, 10)
        self.timer = self.create_timer(1.0 / max(0.1, hz), self._tick)

        self.get_logger().info(f"ReferencePathPublisher publishing: {self.out_topic}")
        self.get_logger().info(f"CSV: {csv_path} poses: {len(self.path_msg.poses)} frame: {self.frame_id}")
        self.get_logger().info(f"swap_unity_to_ros={self.swap_axes} use_2d={self.use_2d}")

    def _build_path_from_csv(self, csv_path: str) -> Path:
        path = Path()
        path.header.frame_id = self.frame_id

        with open(csv_path, newline='') as f:
            reader = csv.DictReader(f)
            required = {'seq', 'pos_x', 'pos_y', 'pos_z', 'image_rel'}
            if not required.issubset(set(reader.fieldnames or [])):
                raise RuntimeError(f"CSV header mismatch. Need {required}, got {reader.fieldnames}")

            for row in reader:
                px = float(row['pos_x'])
                py = float(row['pos_y'])
                pz = float(row['pos_z'])

                ps = PoseStamped()
                ps.header.frame_id = self.frame_id

                if self.swap_axes:
                    ps.pose.position.x = px
                    ps.pose.position.y = pz
                    ps.pose.position.z = 0.0 if self.use_2d else py
                else:
                    ps.pose.position.x = px
                    ps.pose.position.y = py
                    ps.pose.position.z = 0.0 if self.use_2d else pz

                ps.pose.orientation.w = 1.0
                path.poses.append(ps)

        return path

    def _tick(self):
        stamp = self.get_clock().now().to_msg()
        self.path_msg.header.stamp = stamp
        for ps in self.path_msg.poses:
            ps.header.stamp = stamp
        self.pub.publish(self.path_msg)

def main():
    rclpy.init()
    node = ReferencePathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
