#!/usr/bin/env python3
import os
import signal
import subprocess
import sys

def main():
    base_dir = os.path.dirname(os.path.abspath(__file__))

    ref_script  = os.path.join(base_dir, "reference_path_publisher.py")
    live_script = os.path.join(base_dir, "live_path_publisher.py")

    for p in [ref_script, live_script]:
        if not os.path.exists(p):
            print(f"[ERROR] Missing script: {p}")
            sys.exit(1)

    # Defaults (env ile override edebilirsin)
    csv_path   = os.environ.get("CSV", "/home/eren/bitirme_repo/bitirme_dataset/references.csv")
    frame_id   = os.environ.get("FRAME", "map")

    # Live kaynak seçimi:
    # - PoseStamped için: POSE_TYPE=pose  LIVE_TOPIC=/drone/ground_truth_pose
    # - Float32MultiArray için: POSE_TYPE=array LIVE_TOPIC=/drone/sift_pose
    pose_type  = os.environ.get("POSE_TYPE", "pose")
    live_topic = os.environ.get("LIVE_TOPIC", "/drone/ground_truth_pose")

    if not os.path.exists(csv_path):
        print(f"[ERROR] CSV not found: {csv_path}")
        sys.exit(1)

    print("[INFO] CSV:", csv_path)
    print("[INFO] frame_id:", frame_id)
    print("[INFO] live pose_type:", pose_type)
    print("[INFO] live input topic:", live_topic)

    ref_cmd = [
        "python3", ref_script,
        "--ros-args",
        "-p", f"reference_csv_path:={csv_path}",
        "-p", "output_topic:=/trajectory/reference",
        "-p", f"frame_id:={frame_id}",
        "-p", "publish_hz:=1.0",
        "-p", "swap_unity_to_ros:=true",
        "-p", "use_2d:=true",
    ]

    live_cmd = [
        "python3", live_script,
        "--ros-args",
        "-p", f"pose_type:={pose_type}",
        "-p", f"input_topic:={live_topic}",
        "-p", "output_topic:=/trajectory/live",
        "-p", f"frame_id:={frame_id}",
        "-p", "max_points:=5000",
        # array modunda işe yarar:
        "-p", "min_conf:=0.35",
        "-p", "swap_unity_to_ros:=true",
        "-p", "use_2d:=true",
    ]

    procs = [
        subprocess.Popen(ref_cmd),
        subprocess.Popen(live_cmd),
    ]

    def shutdown(*_):
        print("\n[INFO] Shutting down...")
        for p in procs:
            if p.poll() is None:
                p.send_signal(signal.SIGINT)
        for p in procs:
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                p.kill()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    exit_codes = [p.wait() for p in procs]
    print("[INFO] Exit codes:", exit_codes)
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
