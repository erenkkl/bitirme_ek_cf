#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray

class LivePathPublisher(Node):
    def __init__(self):
        super().__init__('live_path_publisher')

        self.declare_parameter('pose_type', 'pose')  # 'pose' or 'array'
        self.declare_parameter('input_topic', '/drone/ground_truth_pose')
        self.declare_parameter('output_topic', '/trajectory/live')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('max_points', 5000)

        # array modunda confidence gating
        self.declare_parameter('min_conf', 0.35)

        # Unity->ROS axis swap (array modunda anlamlı; pose modunda genelde zaten ROS frame olur)
        self.declare_parameter('swap_unity_to_ros', False)
        self.declare_parameter('use_2d', True)

        self.pose_type = self.get_parameter('pose_type').value
        self.in_topic = self.get_parameter('input_topic').value
        self.out_topic = self.get_parameter('output_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.max_points = int(self.get_parameter('max_points').value)

        self.min_conf = float(self.get_parameter('min_conf').value)
        self.swap_axes = bool(self.get_parameter('swap_unity_to_ros').value)
        self.use_2d = bool(self.get_parameter('use_2d').value)

        self.path = Path()
        self.path.header.frame_id = self.frame_id

        self.pub = self.create_publisher(Path, self.out_topic, 10)

        if self.pose_type == 'pose':
            self.sub = self.create_subscription(PoseStamped, self.in_topic, self.cb_pose, 10)
        elif self.pose_type == 'array':
            self.sub = self.create_subscription(Float32MultiArray, self.in_topic, self.cb_array, 10)
        else:
            raise RuntimeError("pose_type must be 'pose' or 'array'")

        self.get_logger().info(
            f"LivePathPublisher started type={self.pose_type} {self.in_topic} -> {self.out_topic} "
            f"frame={self.frame_id} max_points={self.max_points}"
        )

    def _append_and_publish(self, ps: PoseStamped):
        self.path.header.frame_id = self.frame_id
        self.path.header.stamp = ps.header.stamp
        self.path.poses.append(ps)

        if len(self.path.poses) > self.max_points:
            self.path.poses = self.path.poses[-self.max_points:]

        self.pub.publish(self.path)

    def cb_pose(self, msg: PoseStamped):
        # PoseStamped zaten ROS frame’inde geliyorsa swap yapma (varsayılan swap_unity_to_ros=False)
        ps = PoseStamped()
        ps.header.frame_id = self.frame_id
        ps.header.stamp = self.get_clock().now().to_msg()

        if self.use_2d:
            ps.pose = msg.pose
            ps.pose.position.z = 0.0
        else:
            ps.pose = msg.pose

        self._append_and_publish(ps)

    def cb_array(self, msg: Float32MultiArray):
        data = msg.data
        if data is None or len(data) < 5:
            return

        x = float(data[0])
        y = float(data[1])
        z = float(data[2])
        conf = float(data[4])

        if conf < self.min_conf:
            return

        ps = PoseStamped()
        ps.header.frame_id = self.frame_id
        ps.header.stamp = self.get_clock().now().to_msg()

        if self.swap_axes:
            ps.pose.position.x = x
            ps.pose.position.y = z
            ps.pose.position.z = 0.0 if self.use_2d else y
        else:
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = 0.0 if self.use_2d else z

        ps.pose.orientation.w = 1.0
        self._append_and_publish(ps)

def main():
    rclpy.init()
    node = LivePathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
