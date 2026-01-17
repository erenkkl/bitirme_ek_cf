#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String
from cv_bridge import CvBridge
import cv2
import numpy as np
import csv
import os
import time
import threading

# ==========================================
#        CONFIG
# ==========================================

LOG_INTERVAL = 3.0
CONFIDENCE_SATURATION = 35.0
SEARCH_WINDOW_FORWARD = 15   # MAP mode: search ahead (drone moving forward)
SEARCH_WINDOW_BACKWARD = 10  # MAP mode: search behind
MIN_MATCH_COUNT = 10

# ==========================================

class OrbLocalizerNode(Node):
    def __init__(self):
        super().__init__('orb_localizer_node')

        self.declare_parameter('reference_csv_path', '/home/eren/bitirme_repo/bitirme_dataset/references.csv')
        self.declare_parameter('image_topic', '/drone/camera/bottom')
        self.declare_parameter('pose_topic', '/drone/sift_pose')
        self.declare_parameter('set_db_topic', '/drone/sift/set_db')
        self.declare_parameter('debug_viz', False)

        self.ref_csv_path_map = self.get_parameter('reference_csv_path').get_parameter_value().string_value
        self.ref_csv_path_history = os.path.join(os.path.expanduser("~"), "/home/eren/bitirme_repo/flight_dataset/references.csv")

        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        self.set_db_topic = self.get_parameter('set_db_topic').get_parameter_value().string_value
        self.show_debug = self.get_parameter('debug_viz').get_parameter_value().bool_value

        # ORB & Matcher
        self.orb = cv2.ORB_create(nfeatures=2000)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
        self.bridge = CvBridge()

        self.reference_db = []
        self.last_seq = 0
        self.db_mode = "MAP"
        self.db_lock = threading.Lock()
        self.last_log_time = 0.0
        self.last_inliers = 0

        # Backtrack stability lock
        self.stability_seq = None
        self.stability_counter = 0
        self.stability_threshold = 5  # Lock after N consecutive matches
        self.locked_max_seq = None

        # Image dimensions
        self.img_w = 320
        self.img_h = 240
        self.ref_w = 640
        self.ref_h = 480

        # Database caching
        self.db_map = []
        self.db_history = []

        self.get_logger().info("Loading Main Map...")
        self.db_map = self.load_reference_list(self.ref_csv_path_map)
        self.reference_db = self.db_map

        self.sub = self.create_subscription(Image, self.image_topic, self.image_callback, 1)
        self.sub_db = self.create_subscription(String, self.set_db_topic, self.set_db_callback, 10)
        self.pub = self.create_publisher(Float32MultiArray, self.pose_topic, 10)

        self.get_logger().info(f"ORB Localizer Started | Log:{LOG_INTERVAL}s | ConfSat:{CONFIDENCE_SATURATION} | Window:[+{SEARCH_WINDOW_FORWARD}/-{SEARCH_WINDOW_BACKWARD}]")

    def set_db_callback(self, msg):
        mode = msg.data.upper()
        if mode == self.db_mode:
            return

        self.get_logger().info(f"Switching Database: {self.db_mode} -> {mode}")

        with self.db_lock:
            if mode == "MAP":
                start_seq = 0
                if self.db_mode == "HISTORY" and self.last_seq > 0:
                    for item in self.reference_db:
                        if item['seq'] == self.last_seq:
                            start_seq = int(item.get('map_seq', 0))
                            self.get_logger().info(f"Smart Relock: History Seq {self.last_seq} -> Map Seq {start_seq}")
                            break

                self.reference_db = self.db_map
                self.db_mode = "MAP"
                self.last_seq = start_seq
                self.get_logger().info(f"Switched to MAP (Cached) | Start Seq: {self.last_seq}")

            elif mode == "HISTORY":
                self.db_history = self.load_reference_list(self.ref_csv_path_history)
                self.reference_db = self.db_history
                self.db_mode = "HISTORY"

                if len(self.reference_db) > 0:
                    self.last_seq = self.reference_db[-1]['seq']
                else:
                    self.last_seq = 0

                # Reset stability lock when entering HISTORY mode
                self.stability_seq = None
                self.stability_counter = 0
                self.locked_max_seq = None

                self.get_logger().info(f"Switched to HISTORY (Reloaded) | Size: {len(self.reference_db)} | LastSeq: {self.last_seq}")

            else:
                self.get_logger().warn(f"Unknown DB Mode: {mode}")

    def load_reference_list(self, csv_path):
        new_db = []
        if not os.path.exists(csv_path):
            self.get_logger().error(f"Reference file not found: {csv_path}")
            return new_db

        dataset_dir = os.path.dirname(csv_path)
        try:
            with open(csv_path, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    img_path = os.path.join(dataset_dir, row['image_rel'])
                    if os.path.exists(img_path):
                        img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
                        if img is not None:
                            kp, des = self.orb.detectAndCompute(img, None)
                            if des is not None and len(kp) > 5:
                                rec_conf = float(row.get('recorded_conf', 1.0))
                                map_seq = float(row.get('original_match_seq', 0.0))

                                new_db.append({
                                    'seq': int(row['seq']),
                                    'kp': kp,
                                    'des': des,
                                    'rec_conf': rec_conf,
                                    'map_seq': map_seq
                                })
        except Exception as e:
            self.get_logger().error(f"DB Load Error: {e}")

        new_db.sort(key=lambda x: x['seq'])
        return new_db

    def compute_match(self, kp_live, des_live, target_kp, target_des):
        if des_live is None or target_des is None:
            return 0, 0.0, 0.0, None

        # ORB matching with BFMatcher
        matches = self.matcher.knnMatch(des_live, target_des, k=2)
        good = []
        for match_pair in matches:
            if len(match_pair) == 2:
                m, n = match_pair
                if m.distance < 0.75 * n.distance:
                    good.append(m)

        inliers = 0
        off_x, off_y = 0.0, 0.0
        H = None

        if len(good) >= 4:
            src_pts = np.float32([kp_live[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
            dst_pts = np.float32([target_kp[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)

            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
            if M is not None:
                H = M
                inliers = np.sum(mask)
                if inliers > 8:
                    center_point = np.array([[[self.img_w / 2.0, self.img_h / 2.0]]], dtype=np.float32)
                    projected_center = cv2.perspectiveTransform(center_point, M)
                    proj_x = projected_center[0][0][0]
                    proj_y = projected_center[0][0][1]

                    ref_center_x = self.ref_w / 2.0
                    ref_center_y = self.ref_h / 2.0

                    off_x = proj_x - ref_center_x
                    off_y = proj_y - ref_center_y

        return inliers, off_x, off_y, H

    def extract_rotation_from_homography(self, H):
        if H is None:
            return 0.0

        try:
            rotation_rad = np.arctan2(H[1, 0], H[0, 0])
            rotation_deg = np.degrees(rotation_rad)
            rotation_deg = (rotation_deg + 180) % 360 - 180
            return rotation_deg
        except:
            return 0.0

    def get_candidates(self):
        if self.db_mode == "HISTORY":
            backward_window = 4  # Search more toward lower seq (where we're going)
            forward_window = 2    # Allow slight forward search for noise/drift
            start_idx = None
            end_idx = None

            for i, ref in enumerate(self.reference_db):
                if ref['seq'] >= self.last_seq - backward_window and start_idx is None:
                    start_idx = i
                if ref['seq'] > self.last_seq + forward_window:
                    end_idx = i
                    break

            if start_idx is None:
                start_idx = 0
            if end_idx is None:
                end_idx = len(self.reference_db)

            candidates = self.reference_db[start_idx:end_idx]
            return list(reversed(candidates))

        # MAP mode: Asymmetric search (wider forward, narrower backward)
        start_idx = 0
        end_idx = len(self.reference_db)

        for i, ref in enumerate(self.reference_db):
            if ref['seq'] <= self.last_seq - SEARCH_WINDOW_BACKWARD:
                start_idx = i
            if ref['seq'] > self.last_seq + SEARCH_WINDOW_FORWARD:
                end_idx = i
                break

        return self.reference_db[start_idx:end_idx]

    def image_callback(self, msg):
        start_t = time.time()

        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.ref_h, self.ref_w = cv_img.shape[:2]

            cv_img_small = cv2.resize(cv_img, (0, 0), fx=0.5, fy=0.5)
            gray = cv2.cvtColor(cv_img_small, cv2.COLOR_BGR2GRAY)
            self.img_h, self.img_w = cv_img_small.shape[:2]

            kp_live, des_live = self.orb.detectAndCompute(gray, None)
            if des_live is None:
                return
        except:
            return

        pub_seq = -1.0
        pub_off_x = 0.0
        pub_off_y = 0.0
        pub_inliers = 0
        pub_conf = 0.0
        pub_recorded_conf = 0.0
        pub_rotation = 0.0

        best_match = None
        max_inliers = 0
        best_offset = (0.0, 0.0)
        best_H = None

        with self.db_lock:
            search_list = self.get_candidates()
            if len(search_list) == 0 and len(self.reference_db) > 0:
                search_list = self.reference_db[::5]

            for ref in search_list:
                # HISTORY mode: Apply stability lock
                if self.db_mode == "HISTORY" and self.locked_max_seq is not None:
                    if ref['seq'] > self.locked_max_seq:
                        continue  # Skip sequences higher than locked max

                inl, ox, oy, H = self.compute_match(kp_live, des_live, ref['kp'], ref['des'])
                if inl > max_inliers:
                    max_inliers = inl
                    best_match = ref
                    best_offset = (ox, oy)
                    best_H = H

        if max_inliers >= MIN_MATCH_COUNT:
            self.last_seq = best_match['seq']

            pub_seq = float(self.last_seq)
            pub_off_x = best_offset[0]
            pub_off_y = best_offset[1]
            pub_inliers = max_inliers
            pub_conf = min(1.0, max_inliers / float(CONFIDENCE_SATURATION))

            pub_recorded_conf = best_match.get('rec_conf', 1.0 if self.db_mode == "MAP" else 0.0)

            pub_rotation = self.extract_rotation_from_homography(best_H)

            if self.db_mode == "HISTORY":
                # Stability lock mechanism: track consecutive matches
                if self.stability_seq == self.last_seq:
                    # Same seq matched again
                    self.stability_counter += 1
                    if self.stability_counter >= self.stability_threshold and self.locked_max_seq is None:
                        # Lock achieved!
                        self.locked_max_seq = self.last_seq
                        self.get_logger().info(f"[HISTORY] Stability Lock! Max Seq: {self.locked_max_seq}")
                else:
                    # Different seq matched, reset counter
                    self.stability_seq = self.last_seq
                    self.stability_counter = 1

                # Normalize rotation around 180 for backtrack
                pub_rotation += 180.0
                pub_rotation = (pub_rotation + 180.0) % 360.0 - 180.0

                # Convert to deviation from ideal 180
                if pub_rotation > 0:
                    pub_rotation = 180.0 - pub_rotation
                else:
                    pub_rotation = -180.0 - pub_rotation

        self.last_inliers = max_inliers

        # Calculate processing time before publishing
        proc_time_ms = (time.time() - start_t) * 1000.0

        # Publish
        msg_pub = Float32MultiArray()
        # [Seq, OffX, OffY, Conf, ModeID (MAP=0, HIST=1), Inliers, RecordedConf, Rotation, MatchDuration_ms]
        mode_id = 0.0 if self.db_mode == "MAP" else 1.0
        msg_pub.data = [float(pub_seq), float(pub_off_x), float(pub_off_y), float(pub_conf),
                        mode_id, float(pub_inliers), float(pub_recorded_conf), float(pub_rotation),
                        float(proc_time_ms)]
        self.pub.publish(msg_pub)

        # Logging
        current_time = time.time()

        if current_time - self.last_log_time > LOG_INTERVAL:
            self.last_log_time = current_time
            if pub_seq != -1:
                self.get_logger().info(
                    f"[{self.db_mode}] Seq:{int(pub_seq)} | Inliers:{max_inliers} | Conf:{pub_conf:.2f} | Time:{proc_time_ms:.1f}ms"
                )
            else:
                self.get_logger().warn(f"[{self.db_mode}:LOST] No Match | Time:{proc_time_ms:.1f}ms")

        if self.show_debug:
            self.visualize_debug(cv_img, pub_seq, pub_off_x, pub_inliers, proc_time_ms)

    def visualize_debug(self, img, seq, ox, inl, dt_ms):
        if seq != -1:
            txt = f"Seq: {int(seq)} | Inl: {inl}"
            color = (0, 255, 0)
            threshold = 5.0
            if ox < -threshold:
                cv2.putText(img, "<<< LEFT", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            elif ox > threshold:
                cv2.putText(img, "RIGHT >>>", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            else:
                cv2.putText(img, "CENTER", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        else:
            txt = "LOST"
            color = (0, 0, 255)

        cv2.putText(img, f"{self.db_mode}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
        cv2.putText(img, txt, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        cv2.putText(img, f"{dt_ms:.1f} ms", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        cv2.imshow("ORB Localizer", img)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(OrbLocalizerNode())
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
