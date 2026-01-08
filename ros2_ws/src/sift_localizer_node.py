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

# ==========================================
#        ⚙️ AYARLAR KÖŞESİ (CONFIG) ⚙️
# ==========================================

# 1. LOGLAMA SIKLIĞI (Saniye)
# Terminali spamlamamak için kaç saniyede bir log basılsın?
# 0.0 = Her karede basar (Hızlı)
# 0.2 = Saniyede 5 kere basar (Okunaklı)
# 1.0 = Saniyede 1 kere basar (Sakin)
LOG_INTERVAL = 1.0  

# 2. GÜVEN DOYGUNLUĞU (Confidence Saturation)
# Kaç tane sağlam nokta (Inlier) bulursak güven %100 (1.0) olsun?
# Düşük (20) = Çabuk güvenir | Yüksek (60) = Zor beğenir
CONFIDENCE_SATURATION = 35.0

# 3. ARAMA PENCERESİ (Search Window)
# Son bulunan konumun +/- kaç kare ötesine bakalım?
# Küçük (10) = Hızlı ama kaçırabilir | Büyük (50) = Yavaş ama güvenli
SEARCH_WINDOW = 20

# 4. MİNİMUM EŞLEŞME
MIN_MATCH_COUNT = 10 

# ==========================================

class SiftLocalizerNode(Node):
    def __init__(self):
        super().__init__('sift_localizer_node')

        self.declare_parameter('reference_csv_path', '/home/eren/bitirme_repo/bitirme_dataset/references.csv')
        self.declare_parameter('image_topic', '/drone/camera/bottom')
        self.declare_parameter('pose_topic', '/drone/sift_pose')
        self.declare_parameter('debug_viz', False)

        self.ref_csv_path = self.get_parameter('reference_csv_path').get_parameter_value().string_value
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        self.show_debug = self.get_parameter('debug_viz').get_parameter_value().bool_value

        # SIFT & Matcher
        self.sift = cv2.SIFT_create(nfeatures=1000)
        index_params = dict(algorithm=1, trees=5) 
        search_params = dict(checks=50)
        self.flann = cv2.FlannBasedMatcher(index_params, search_params)
        self.bridge = CvBridge()
        
        self.reference_db = []
        self.last_seq = 0 
        
        # Loglama Zamanlayıcısı
        self.last_log_time = 0.0
        
        
        self.img_w = 320    # Canlı (Küçük) Resim
        self.img_h = 240
        self.ref_w = 640    # Referans (Büyük) Resim
        self.ref_h = 480

        self.load_reference_db()
        self.sub = self.create_subscription(Image, self.image_topic, self.image_callback, 1)
        self.pub = self.create_publisher(Float32MultiArray, self.pose_topic, 10)
        self.get_logger().info(f"SiftLocalizer Started. Log:{LOG_INTERVAL}s | ConfSat:{CONFIDENCE_SATURATION} | Win:{SEARCH_WINDOW}")

    def load_reference_db(self):
        if not os.path.exists(self.ref_csv_path): return
        dataset_dir = os.path.dirname(self.ref_csv_path)
        try:
            with open(self.ref_csv_path, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    img_path = os.path.join(dataset_dir, row['image_rel'])
                    if os.path.exists(img_path):
                        img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
                        if img is not None:
                            kp, des = self.sift.detectAndCompute(img, None)
                            if des is not None and len(kp) > 5:
                                self.reference_db.append({'seq': int(row['seq']), 'kp': kp, 'des': des.astype(np.float32)})
        except: pass
        self.reference_db.sort(key=lambda x: x['seq'])
        if len(self.reference_db) > 0:
            first_seq = self.reference_db[0]['seq']
            last_seq = self.reference_db[-1]['seq']
            self.get_logger().info(f"Referans gorseller yuklendi: {len(self.reference_db)} Gorsel (Seq: {first_seq} -> {last_seq})")
        else:
            self.get_logger().error(f"Referans dosyasi bos! Dosya yolu hatali olabilir: {self.ref_csv_path}")
    def compute_match(self, kp_live, des_live, target_kp, target_des):
        """MERKEZ İZDÜŞÜMÜ VE HATA HESABI"""
        if des_live is None or target_des is None: return 0, 0.0, 0.0

        matches = self.flann.knnMatch(des_live, target_des, k=2)
        good = []
        for m, n in matches:
            if m.distance < 0.75 * n.distance: good.append(m)

        inliers = 0
        off_x, off_y = 0.0, 0.0

        if len(good) >= 4:
            src_pts = np.float32([kp_live[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
            dst_pts = np.float32([target_kp[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)

            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
            if M is not None:
                inliers = np.sum(mask)
                if inliers > 8:
                    # 1. CANLI MERKEZ (Küçük)
                    center_point = np.array([[[self.img_w/2.0, self.img_h/2.0]]], dtype=np.float32)
                    
                    # 2. IŞINLAMA
                    projected_center = cv2.perspectiveTransform(center_point, M)
                    proj_x = projected_center[0][0][0]
                    proj_y = projected_center[0][0][1]
                    
                    # 3. REFERANS MERKEZİ (Büyük)
                    ref_center_x = self.ref_w / 2.0
                    ref_center_y = self.ref_h / 2.0
                    
                    # 4. HATA
                    off_x = proj_x - ref_center_x
                    off_y = proj_y - ref_center_y

        return inliers, off_x, off_y

    def get_candidates(self):
        # Arama penceresi parametresini kullan
        window = SEARCH_WINDOW
        start = max(0, self.last_seq - window)
        end = min(len(self.reference_db), self.last_seq + window)
        return self.reference_db[start:end]

    def image_callback(self, msg):
        start_t = time.time() 

        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.ref_h, self.ref_w = cv_img.shape[:2] 

            # Resize
            cv_img_small = cv2.resize(cv_img, (0,0), fx=0.5, fy=0.5)
            gray = cv2.cvtColor(cv_img_small, cv2.COLOR_BGR2GRAY)
            self.img_h, self.img_w = cv_img_small.shape[:2] 
            
            kp_live, des_live = self.sift.detectAndCompute(gray, None)
            if des_live is None: return
            des_live = des_live.astype(np.float32)
        except: return

        # Varsayılanlar
        pub_seq = -1.0; pub_off_x = 0.0; pub_off_y = 0.0
        pub_inliers = 0; pub_conf = 0.0

        best_match = None
        max_inliers = 0
        best_offset = (0.0, 0.0)

        # Arama
        search_list = self.get_candidates()
        if len(search_list) == 0: search_list = self.reference_db[::5] 

        for ref in search_list:
            inl, ox, oy = self.compute_match(kp_live, des_live, ref['kp'], ref['des'])
            if inl > max_inliers:
                max_inliers = inl
                best_match = ref
                best_offset = (ox, oy)

        if max_inliers >= MIN_MATCH_COUNT:
            self.last_seq = best_match['seq']
            
            pub_seq = float(self.last_seq)
            pub_off_x = best_offset[0]
            pub_off_y = best_offset[1]
            pub_inliers = max_inliers
            # Güven Doygunluğu Parametresini Kullan
            pub_conf = min(1.0, max_inliers / float(CONFIDENCE_SATURATION))
        else:
            pub_seq = -1.0

        # --- YAYIN ---
        msg_pub = Float32MultiArray()
        msg_pub.data = [float(pub_seq), float(pub_off_x), float(pub_off_y), float(pub_conf), 0.0, float(pub_inliers)]
        self.pub.publish(msg_pub)

        # --- LOGLAMA MANTIĞI ---
        proc_time_ms = (time.time() - start_t) * 1000.0
        current_time = time.time()
        
        if current_time - self.last_log_time > LOG_INTERVAL:
            self.last_log_time = current_time
            if pub_seq != -1:
                self.get_logger().info(f"[MATCH] Seq:{int(pub_seq)} | Inl:{max_inliers} | Conf:{pub_conf:.2f} | Time:{proc_time_ms:.1f}ms")
            else:
                self.get_logger().warn(f"[LOST]  No Match | Time:{proc_time_ms:.1f}ms")

        if self.show_debug:
            self.visualize_debug(cv_img, pub_seq, pub_off_x, pub_inliers, proc_time_ms)

    def visualize_debug(self, img, seq, ox, inl, dt_ms):
        if seq != -1:
            txt = f"Seq: {int(seq)} | Inl: {inl}"
            color = (0, 255, 0)
            threshold = 5.0
            if ox < -threshold:
                cv2.putText(img, "<<< SOLDA", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            elif ox > threshold:
                cv2.putText(img, "SAGDA >>>", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            else:
                cv2.putText(img, "MERKEZ", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        else:
            txt = "LOST"
            color = (0, 0, 255)

        cv2.putText(img, txt, (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        cv2.putText(img, f"{dt_ms:.1f} ms", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        cv2.imshow("Sift V8.3 Configurable", img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SiftLocalizerNode())
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
