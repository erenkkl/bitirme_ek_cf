#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
import numpy as np
import time

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

class DepthObstacleAvoider(Node):
    """
    Publishes /drone/avoid_cmd as Float32MultiArray:
      [active, pitch_deg, roll_deg, yaw_cmd, lost_request]

    active=0 => Unity arbiter SIFT'e kontrolü bırakır
    active=1 => Unity arbiter SIFT'i susturur, OA komutunu uygular
    """

    def __init__(self):
        super().__init__('depth_obstacle_avoidance_node')

        # Topics
        self.depth_topic = self.declare_parameter('depth_topic', '/front_depth/image_raw').value
        self.cmd_topic   = self.declare_parameter('cmd_topic',   '/drone/avoid_cmd').value

        # Sector / ROI
        self.sectors = int(self.declare_parameter('sectors', 15).value)
        self.roi_y0  = float(self.declare_parameter('roi_y0', 0.20).value)
        self.roi_y1  = float(self.declare_parameter('roi_y1', 0.55).value)

        # Activation / deactivation (hysteresis)
        self.slow_distance  = float(self.declare_parameter('slow_distance', 6.0).value)
        self.clear_distance = float(self.declare_parameter('clear_distance', 8.0).value)
        self.stop_distance  = float(self.declare_parameter('stop_distance', 2.5).value)

        # Command limits
        self.max_pitch_deg = float(self.declare_parameter('max_pitch_deg', 6.0).value)
        self.max_roll_deg  = float(self.declare_parameter('max_roll_deg', 18.0).value)
        self.max_yaw_cmd   = float(self.declare_parameter('max_yaw_cmd', 8.0).value)

        # Gains
        self.k_roll = float(self.declare_parameter('k_roll', 14.0).value)
        self.k_yaw  = float(self.declare_parameter('k_yaw',  10.0).value)
        self.k_fwd  = float(self.declare_parameter('k_fwd',   2.5).value)

        # Risk scaling
        self.risk_min_scale = float(self.declare_parameter('risk_min_scale', 0.5).value)
        self.risk_max_scale = float(self.declare_parameter('risk_max_scale', 1.2).value)

        # Dynamic inflation (body radius + buffer)
        self.base_inflation  = float(self.declare_parameter('base_inflation', 0.9).value)
        self.extra_inflation = float(self.declare_parameter('extra_inflation', 0.7).value)

        # Wall-follow (yan mesafe koruma)
        self.side_safe_distance = float(self.declare_parameter('side_safe_distance', 1.5).value)  # metre
        self.side_kp = float(self.declare_parameter('side_kp', 0.45).value)  # steer düzeltmesi (0..~1)

        # Smoothing
        self.alpha = float(self.declare_parameter('cmd_smoothing', 0.35).value)
        self._prev_pitch = 0.0
        self._prev_roll  = 0.0
        self._prev_yaw   = 0.0

        # Debounce
        self.clear_frames_needed = int(self.declare_parameter('clear_frames_needed', 6).value)
        self._clear_counter = 0
        self._avoid_active = False

        # Valid depth
        self.min_valid_ratio = float(self.declare_parameter('min_valid_ratio', 0.02).value)

        # Stuck scan
        self.stuck_yaw_scan_rate = float(self.declare_parameter('stuck_yaw_scan_rate', 0.6).value)
        self.stuck_clearance_threshold = float(self.declare_parameter('stuck_clearance_threshold', 0.25).value)

        # Front cone + side-hold
        self.front_half_width = int(self.declare_parameter('front_half_width', 2).value)   # center±k
        self.side_hold_sec = float(self.declare_parameter('side_hold_sec', 1.5).value)
        self.min_abs_steer = float(self.declare_parameter('min_abs_steer', 0.35).value)

        self._avoid_side = 0
        self._side_hold_until = 0.0

        self.sub = self.create_subscription(Image, self.depth_topic, self.on_depth, 10)
        self.pub = self.create_publisher(Float32MultiArray, self.cmd_topic, 10)

        self.get_logger().info(
            f"[OA] depth={self.depth_topic} cmd={self.cmd_topic} "
            f"slow={self.slow_distance} clear={self.clear_distance} stop={self.stop_distance} "
            f"infl(base={self.base_inflation},extra={self.extra_inflation}) side_safe={self.side_safe_distance}"
        )

    def on_depth(self, msg: Image):
        if msg.encoding != '32FC1':
            self.get_logger().warn_once(f"[OA] Expected 32FC1, got {msg.encoding}")
            return

        h = int(msg.height)
        w = int(msg.width)
        depth = np.frombuffer(msg.data, dtype=np.float32)
        if depth.size != h * w:
            return
        depth = depth.reshape((h, w))

        # ROI
        y0 = int(clamp(self.roi_y0, 0.0, 0.99) * h)
        y1 = int(clamp(self.roi_y1, 0.01, 1.0) * h)
        if y1 <= y0 + 5:
            y0 = int(0.20 * h)
            y1 = int(0.55 * h)

        roi = depth[y0:y1, :]
        valid = np.isfinite(roi) & (roi > 0.02) & (roi < 500.0)
        valid_ratio = float(valid.sum()) / float(valid.size)

        if valid_ratio < self.min_valid_ratio:
            self._avoid_active = False
            self._clear_counter = 0
            self._avoid_side = 0
            self._side_hold_until = 0.0
            self.publish_cmd(0.0, 0.0, 0.0, 0.0, 0.0)
            return

        # Sectors
        sectors = max(3, self.sectors)
        sector_w = w // sectors
        if sector_w < 4:
            self.publish_cmd(0.0, 0.0, 0.0, 0.0, 0.0)
            return

        clearances = np.zeros((sectors,), dtype=np.float32)
        for i in range(sectors):
            x0 = i * sector_w
            x1 = w if i == sectors - 1 else (i + 1) * sector_w
            patch = roi[:, x0:x1]
            vpatch = valid[:, x0:x1]
            vals = patch[vpatch]
            clearances[i] = 0.0 if vals.size == 0 else np.percentile(vals, 10)

        center = sectors // 2

        # Front cone distance: center±k
        k = max(1, self.front_half_width)
        a = max(0, center - k)
        b = min(sectors - 1, center + k)
        d_front = float(np.min(clearances[a:b+1]))

        # Global min distance (ignore zeros)
        nz = clearances[clearances > 0.01]
        d_min = float(np.min(nz)) if nz.size > 0 else 999.0

        now = time.time()

        # --- Activation / deactivation ---
        if not self._avoid_active:
            if (d_front < self.slow_distance) or (d_min < self.slow_distance):
                self._avoid_active = True
                self._clear_counter = 0
                # choose initial side: compare max clearance on each side
                left_clear  = float(np.max(clearances[:center])) if center > 0 else 0.0
                right_clear = float(np.max(clearances[center+1:])) if center < sectors - 1 else 0.0
                self._avoid_side = 1 if right_clear >= left_clear else -1
                self._side_hold_until = now + self.side_hold_sec
        else:
            # Exit only if front AND global are clear
            if (d_front > self.clear_distance) and (d_min > self.clear_distance):
                # Additional safety: side also clear enough before giving back control
                if self.side_is_clear(clearances, center):
                    self._clear_counter += 1
                    if self._clear_counter >= self.clear_frames_needed:
                        self._avoid_active = False
                        self._clear_counter = 0
                        self._avoid_side = 0
                        self._side_hold_until = 0.0
                else:
                    self._clear_counter = 0
            else:
                self._clear_counter = 0

        if not self._avoid_active:
            self.publish_cmd(0.0, 0.0, 0.0, 0.0, 0.0)
            return

        # --- Risk & dynamic inflation ---
        den = max(1e-6, (self.slow_distance - self.stop_distance))
        risk = clamp((self.slow_distance - d_front) / den, 0.0, 1.0)
        scale = self.risk_min_scale + (self.risk_max_scale - self.risk_min_scale) * risk

        infl = self.base_inflation + risk * self.extra_inflation
        eff_clear = np.maximum(clearances - infl, 0.0)

        # --- Direction choice (side-hold) ---
        if now < self._side_hold_until and self._avoid_side != 0:
            if self._avoid_side > 0:
                search = eff_clear[center:]           # center..right
                best_i = center + int(np.argmax(search))
            else:
                search = eff_clear[:center+1]         # left..center
                best_i = int(np.argmax(search))
        else:
            best_i = int(np.argmax(eff_clear))

        max_clear = float(np.max(eff_clear))
        if max_clear < self.stuck_clearance_threshold:
            yaw = self.scan_yaw()
            self.publish_cmd(1.0, 0.0, 0.0, yaw, 0.0)
            return

        # Base steer
        steer = (best_i - center) / max(1, center)

        # Keep at least a minimum steer in the chosen avoid side
        if self._avoid_side != 0 and abs(steer) < self.min_abs_steer:
            steer = self._avoid_side * self.min_abs_steer

        # --- Wall-follow correction: keep side distance >= side_safe_distance ---
        side_dist = self.compute_side_distance(clearances, center)
        if side_dist < 999.0:
            # error>0 => too close -> push away more (increase |steer|)
            err = clamp((self.side_safe_distance - side_dist) / max(1e-6, self.side_safe_distance), 0.0, 1.0)
            steer = clamp(steer + self._avoid_side * (self.side_kp * err), -1.0, 1.0)

        # Commands
        roll = clamp(self.k_roll * steer, -self.max_roll_deg, self.max_roll_deg)
        yaw  = clamp(self.k_yaw  * steer, -self.max_yaw_cmd,  self.max_yaw_cmd)

        if d_front < self.stop_distance:
            pitch = 0.0
        else:
            pitch = clamp(self.k_fwd * (d_front - self.stop_distance), 0.0, self.max_pitch_deg)

        # Risk scaling: closer => more yaw/roll, less pitch
        roll = clamp(roll * scale, -self.max_roll_deg, self.max_roll_deg)
        yaw  = clamp(yaw  * scale, -self.max_yaw_cmd,  self.max_yaw_cmd)
        pitch = pitch * (1.0 - risk)

        # Smooth
        pitch = self._prev_pitch + self.alpha * (pitch - self._prev_pitch)
        roll  = self._prev_roll  + self.alpha * (roll  - self._prev_roll)
        yaw   = self._prev_yaw   + self.alpha * (yaw   - self._prev_yaw)
        self._prev_pitch, self._prev_roll, self._prev_yaw = pitch, roll, yaw

        self.publish_cmd(1.0, pitch, roll, yaw, 0.0)

    def compute_side_distance(self, clearances, center):
        """
        Side distance estimate on the chosen avoid side.
        We take the MIN distance on that side band to be conservative.
        """
        if self._avoid_side > 0:
            band = clearances[center:]          # right side (including center)
        else:
            band = clearances[:center+1]        # left side (including center)

        band = band[band > 0.01]
        if band.size == 0:
            return 999.0
        return float(np.min(band))

    def side_is_clear(self, clearances, center):
        """
        For exiting avoidance, require that side clearance is also safe enough.
        """
        side_dist = self.compute_side_distance(clearances, center)
        # side_dist is raw distance; require it above side_safe_distance + small buffer
        return side_dist > (self.side_safe_distance + 0.3)

    def scan_yaw(self):
        now = time.time()
        phase = int(now * self.stuck_yaw_scan_rate) % 2
        return self.max_yaw_cmd if phase == 0 else -self.max_yaw_cmd

    def publish_cmd(self, active, pitch, roll, yaw, lost):
        msg = Float32MultiArray()
        msg.data = [float(active), float(pitch), float(roll), float(yaw), float(lost)]
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = DepthObstacleAvoider()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

