#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import csv
import os
import time

class SiftLocalizerNodeV2(Node):
    def __init__(self):
        super().__init__('sift_localizer_node')

        # --- Parameters ---
        self.declare_parameter('reference_csv_path', '/home/eren/Bitirme/bitirme_dataset/references.csv')
        self.declare_parameter('image_topic', '/drone/camera/bottom')
        self.declare_parameter('pose_topic', '/drone/sift_pose')
        self.declare_parameter('debug_visualization', False)
        self.declare_parameter('min_match_count', 10)
        self.declare_parameter('search_window', 20)  # +/- window size
        self.declare_parameter('log_every_n', 10)
        self.declare_parameter('log_match_details', True)

        self.ref_csv_path = self.get_parameter('reference_csv_path').get_parameter_value().string_value
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        self.debug_viz = self.get_parameter('debug_visualization').get_parameter_value().bool_value
        self.min_match_count = self.get_parameter('min_match_count').get_parameter_value().integer_value
        self.search_window = self.get_parameter('search_window').get_parameter_value().integer_value
        self.log_every_n = self.get_parameter('log_every_n').get_parameter_value().integer_value
        self.log_match_details = self.get_parameter('log_match_details').get_parameter_value().bool_value

        # --- SIFT & Matcher Setup ---
        self.sift = cv2.SIFT_create()
        # FLANN parameters
        FLANN_INDEX_KDTREE = 1
        index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
        search_params = dict(checks=50)
        self.flann = cv2.FlannBasedMatcher(index_params, search_params)

        self.bridge = CvBridge()
        self.reference_database = [] # List of dicts
        self.last_seq = None
        self.frame_count = 0

        # --- Load Dataset ---
        self.load_reference_data()

        # --- Subscribers & Publishers ---
        self.sub = self.create_subscription(Image, self.image_topic, self.image_callback, 10)
        self.pub = self.create_publisher(Float32MultiArray, self.pose_topic, 10)

        self.get_logger().info("SiftLocalizerNodeV2 started. Waiting for images...")

    def load_reference_data(self):
        if not os.path.exists(self.ref_csv_path):
            self.get_logger().error(f"Reference CSV not found: {self.ref_csv_path}")
            return

        dataset_dir = os.path.dirname(self.ref_csv_path)
        self.get_logger().info(f"Loading dataset from: {dataset_dir}")

        count = 0
        try:
            with open(self.ref_csv_path, 'r') as f:
                reader = csv.DictReader(f)
                # Header check: seq,pos_x,pos_y,pos_z,image_rel
                
                for row in reader:
                    try:
                        seq = int(row['seq'])
                        x = float(row['pos_x'])
                        y = float(row['pos_y'])
                        z = float(row['pos_z'])
                        image_rel = row['image_rel']
                        
                        image_abs = os.path.join(dataset_dir, image_rel)

                        if not os.path.exists(image_abs):
                            self.get_logger().warn(f"Image missing: {image_abs}")
                            continue

                        # Load grayscale
                        img = cv2.imread(image_abs, cv2.IMREAD_GRAYSCALE)
                        if img is None:
                            self.get_logger().warn(f"Failed to load image: {image_abs}")
                            continue

                        # Compute SIFT
                        kp, des = self.sift.detectAndCompute(img, None)

                        if des is None or len(kp) < 5:
                            continue

                        # Ensure float32 for FLANN
                        if des.dtype != np.float32:
                            des = des.astype(np.float32)

                        ref_entry = {
                            'seq': seq,
                            'x': x, 'y': y, 'z': z,
                            'descriptors': des,
                            'keypoints': kp, # Optional, kept if needed for viz
                            'image_rel': image_rel,
                            'image_abs': image_abs
                            # 'image': img # Uncomment if needed for deep debug
                        }
                        self.reference_database.append(ref_entry)
                        count += 1

                    except ValueError as e:
                        self.get_logger().error(f"CSV parsing error on row: {row} -> {e}")
                        continue
        except Exception as e:
            self.get_logger().error(f"Failed to read CSV: {e}")
            return

        # Sort by seq just in case
        self.reference_database.sort(key=lambda k: k['seq'])
        self.get_logger().info(f"Loaded {count} reference frames.")

    def _iter_candidates(self):
        """Yields reference frames based on search window logic."""
        total_refs = len(self.reference_database)
        if total_refs == 0:
            return

        if self.last_seq is None or self.search_window <= 0:
            # Full scan
            for ref in self.reference_database:
                yield ref
        else:
            # Windowed scan
            # Assumes seq corresponds somewhat to index, but iterating safely is better
            # Optimally we binary search, but linear scan with check is robust enough for small-medium datasets
            min_seq = self.last_seq - self.search_window
            max_seq = self.last_seq + self.search_window
            
            candidates_found = 0
            for ref in self.reference_database:
                if min_seq <= ref['seq'] <= max_seq:
                    yield ref
                    candidates_found += 1
            
            # Fallback if track lost (no candidates in window) -> perform full scan
            if candidates_found == 0:
                self.last_seq = None # Reset
                for ref in self.reference_database:
                    yield ref

    def image_callback(self, msg):
        start_time = time.time()
        self.frame_count += 1
        
        # 1. Convert Image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        except Exception as e:
            self.get_logger().error(f"CV Bridge Error: {e}")
            return

        # 2. SIFT Extraction
        t0 = time.time()
        kp_live, des_live = self.sift.detectAndCompute(gray, None)
        sift_ms = (time.time() - t0) * 1000.0

        if des_live is None or len(kp_live) < self.min_match_count:
            if self.frame_count % self.log_every_n == 0:
                self.get_logger().warn("Not enough keypoints in live image.")
            return

        if des_live.dtype != np.float32:
            des_live = des_live.astype(np.float32)

        # 3. Matching
        t1 = time.time()
        best_match = None
        max_good_matches = 0
        candidate_count = 0

        for ref in self._iter_candidates():
            candidate_count += 1
            # KNN Match
            matches = self.flann.knnMatch(des_live, ref['descriptors'], k=2)
            
            # Ratio Test
            good_matches = []
            for m, n in matches:
                if m.distance < 0.75 * n.distance:
                    good_matches.append(m)
            
            count_good = len(good_matches)
            
            if count_good > max_good_matches:
                max_good_matches = count_good
                best_match = ref

        match_ms = (time.time() - t1) * 1000.0

        # 4. Result Processing
        confidence = 0.0
        published = False
        
        if best_match and max_good_matches >= self.min_match_count:
            confidence = min(1.0, max_good_matches / 60.0)
            
            # Publish
            # data = [x, y, z, seq, confidence, match_count]
            out_msg = Float32MultiArray()
            out_msg.data = [
                float(best_match['x']),
                float(best_match['y']),
                float(best_match['z']),
                float(best_match['seq']),
                float(confidence),
                float(max_good_matches)
            ]
            self.pub.publish(out_msg)
            
            # Update tracking
            self.last_seq = best_match['seq']
            published = True
        else:
            # Lost tracking or poor match
            if self.search_window > 0:
                 # Only reset if we were in window mode and failed
                 # Logic: if best match is terrible, maybe we are totally lost
                 pass 

        # 5. Logging
        total_ms = (time.time() - start_time) * 1000.0
        
        if self.frame_count % self.log_every_n == 0:
            status = "PUBLISHED" if published else "REJECTED"
            best_seq_log = best_match['seq'] if best_match else -1
            
            self.get_logger().info(
                f"[{status}] Frame: {self.frame_count} | KPs: {len(kp_live)} | "
                f"Sift: {sift_ms:.1f}ms | Match: {match_ms:.1f}ms | Tot: {total_ms:.1f}ms | "
                f"Cands: {candidate_count} | BestSeq: {best_seq_log} | Good: {max_good_matches} | Conf: {confidence:.2f}"
            )

            if published and self.log_match_details:
                self.get_logger().info(
                    f"   [MATCHED] live_frame={self.frame_count} -> seq={best_match['seq']} "
                    f"good={max_good_matches} conf={confidence:.2f} "
                    f"ref_rel={best_match['image_rel']} ref_abs={best_match['image_abs']}"
                )

        # 6. Debug Visualization
        if self.debug_viz:
            display_img = cv_image.copy()
            text = f"Seq: {self.last_seq if self.last_seq else 'None'} | Conf: {confidence:.2f}"
            color = (0, 255, 0) if published else (0, 0, 255)
            cv2.putText(display_img, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
            cv2.imshow("Sift Localizer Debug", display_img)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = SiftLocalizerNodeV2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
