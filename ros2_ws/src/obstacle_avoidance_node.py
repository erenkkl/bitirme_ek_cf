#!/usr/bin/env python3
"""
Obstacle Avoidance Decision-Making Node for Drone Navigation

This node acts as an arbiter between SIFT path following and obstacle avoidance.
It implements a state machine: NORMAL → AVOIDING → RECOVERING → NORMAL

Subscribes to:
  - /drone/obstacle_info (Float32MultiArray): Obstacle detection data
  - /drone/sift_pose (Float32MultiArray): SIFT localization data

Publishes to:
  - /drone/avoidance_command (Float32MultiArray): Avoidance commands for path follower

Author: Obstacle Avoidance System
Date: 2026-01-05
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time
import numpy as np


# ==========================================
#        ⚙️ CONFIGURATION PARAMETERS ⚙️
# ==========================================

# State transition thresholds
OBSTACLE_DETECT_RANGE = 15.0        # Enter AVOIDING if obstacle < 15m
OBSTACLE_CLEARED_RANGE = 22.0       # Exit AVOIDING if obstacle > 22m (hysteresis) - INCREASED for thick obstacles
SIFT_CONFIDENCE_THRESHOLD = 0.7     # Min confidence to exit RECOVERING
SIFT_ERROR_THRESHOLD = 20.0         # Max visual error (pixels) to exit RECOVERING

# Avoidance control parameters
AVOIDANCE_ROLL_GAIN = 12.0          # Roll adjustment in degrees during avoidance - INCREASED for more aggressive avoidance
RECOVERY_ROLL_GAIN = 8.0            # Roll adjustment during recovery - INCREASED to 8° for faster path return
AVOIDANCE_SPEED_MULT = 0.6          # Speed reduction during avoidance (60%)
RECOVERY_SPEED_MULT = 0.8           # Speed during recovery (80%)

# Timing constraints
MIN_AVOIDING_DURATION = 2.0         # Min time in AVOIDING before allowing recovery (sec) - INCREASED for thick obstacles
MIN_RECOVERY_DURATION = 2.0         # Min time in RECOVERING before returning to NORMAL (sec)

# Clearance thresholds (NEW)
SIDE_CLEARANCE_THRESHOLD = 0.7      # Min clearance (0-1) on avoiding side before recovery
RECOVERY_COAST_TIME = 0.5           # Time to coast straight before turning back to path (sec) - REDUCED for less drift

# Safety margins
CRITICAL_DISTANCE = 5.0             # Emergency stop distance (meters)
URGENCY_THRESHOLD = 0.8             # High urgency triggers stronger avoidance

# Logging
LOG_INTERVAL = 0.5                  # Log every N seconds


# ==========================================
#        STATE MACHINE DEFINITIONS
# ==========================================

class AvoidanceState:
    NORMAL = 0
    AVOIDING = 1
    RECOVERING = 2
    EMERGENCY_STOP = 3


# ==========================================
#        OBSTACLE AVOIDANCE NODE
# ==========================================

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        # Declare parameters
        self.declare_parameter('obstacle_topic', '/drone/obstacle_info')
        self.declare_parameter('sift_topic', '/drone/sift_pose')
        self.declare_parameter('command_topic', '/drone/avoidance_command')
        self.declare_parameter('debug_mode', False)

        # Get parameters
        obstacle_topic = self.get_parameter('obstacle_topic').get_parameter_value().string_value
        sift_topic = self.get_parameter('sift_topic').get_parameter_value().string_value
        command_topic = self.get_parameter('command_topic').get_parameter_value().string_value
        self.debug_mode = self.get_parameter('debug_mode').get_parameter_value().bool_value

        # Initialize state
        self.current_state = AvoidanceState.NORMAL
        self.state_enter_time = time.time()
        self.last_log_time = time.time()

        # Obstacle data
        self.has_obstacle = False
        self.obstacle_distance = 100.0
        self.preferred_side = 0
        self.obstacle_urgency = 0.0
        self.clear_left = 1.0
        self.clear_right = 1.0

        # SIFT data
        self.sift_seq = -1.0
        self.sift_visual_error = 0.0
        self.sift_confidence = 0.0

        # Avoidance memory
        self.last_avoid_side = 0        # Remember which way we went
        self.sift_confidence_history = []
        self.visual_error_history = []

        # Subscribers
        self.obstacle_sub = self.create_subscription(
            Float32MultiArray,
            obstacle_topic,
            self.obstacle_callback,
            10
        )

        self.sift_sub = self.create_subscription(
            Float32MultiArray,
            sift_topic,
            self.sift_callback,
            10
        )

        # Publisher
        self.command_pub = self.create_publisher(
            Float32MultiArrayMsg := Float32MultiArray,
            command_topic,
            10
        )

        # Control loop timer (20 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(f"ObstacleAvoidanceNode started.")
        self.get_logger().info(f"Config: DetectRange={OBSTACLE_DETECT_RANGE}m, ClearRange={OBSTACLE_CLEARED_RANGE}m")
        self.get_logger().info(f"Avoidance: Roll={AVOIDANCE_ROLL_GAIN}°, Speed={AVOIDANCE_SPEED_MULT*100}%")

    # ==========================================
    #        CALLBACKS
    # ==========================================

    def obstacle_callback(self, msg):
        """Receive obstacle detection data from ObstacleDetector.cs"""
        if len(msg.data) < 6:
            return

        self.has_obstacle = msg.data[0] > 0.5
        self.obstacle_distance = msg.data[1]
        self.preferred_side = int(msg.data[2])
        self.obstacle_urgency = msg.data[3]
        self.clear_left = msg.data[4]
        self.clear_right = msg.data[5]

    def sift_callback(self, msg):
        """Receive SIFT localization data"""
        if len(msg.data) < 4:
            return

        self.sift_seq = msg.data[0]
        self.sift_visual_error = msg.data[1]
        # data[2] is vertical error (unused)
        self.sift_confidence = msg.data[3] if len(msg.data) > 3 else 0.0

        # Update history for recovery monitoring
        self.sift_confidence_history.append(self.sift_confidence)
        if len(self.sift_confidence_history) > 10:
            self.sift_confidence_history.pop(0)

        self.visual_error_history.append(abs(self.sift_visual_error))
        if len(self.visual_error_history) > 10:
            self.visual_error_history.pop(0)

    # ==========================================
    #        CONTROL LOOP (STATE MACHINE)
    # ==========================================

    def control_loop(self):
        """Main control loop - executes state machine at 20 Hz"""

        # State machine transitions
        next_state = self.update_state_machine()

        if next_state != self.current_state:
            self.transition_to_state(next_state)

        # Generate command based on current state
        command = self.generate_command()

        # Publish command
        self.publish_command(command)

        # Logging
        self.log_status()

    def update_state_machine(self):
        """Determine next state based on current state and sensor data"""

        current = self.current_state
        time_in_state = time.time() - self.state_enter_time

        # ========== NORMAL STATE ==========
        if current == AvoidanceState.NORMAL:
            # Check for obstacle
            if self.has_obstacle and self.obstacle_distance < OBSTACLE_DETECT_RANGE:
                # Emergency check
                if self.obstacle_distance < CRITICAL_DISTANCE:
                    return AvoidanceState.EMERGENCY_STOP
                else:
                    return AvoidanceState.AVOIDING

            return AvoidanceState.NORMAL

        # ========== AVOIDING STATE ==========
        elif current == AvoidanceState.AVOIDING:
            # Must stay in AVOIDING for minimum duration
            if time_in_state < MIN_AVOIDING_DURATION:
                return AvoidanceState.AVOIDING

            # NEW: Check both distance AND side clearance before recovery
            # This prevents premature recovery when passing thick obstacles
            avoiding_side_clear = (self.clear_left > SIDE_CLEARANCE_THRESHOLD if self.last_avoid_side == -1
                                   else self.clear_right > SIDE_CLEARANCE_THRESHOLD)

            obstacle_cleared = not self.has_obstacle or self.obstacle_distance > OBSTACLE_CLEARED_RANGE

            if obstacle_cleared and avoiding_side_clear:
                return AvoidanceState.RECOVERING

            # Re-check for emergency
            if self.obstacle_distance < CRITICAL_DISTANCE:
                return AvoidanceState.EMERGENCY_STOP

            return AvoidanceState.AVOIDING

        # ========== RECOVERING STATE ==========
        elif current == AvoidanceState.RECOVERING:
            # Must stay in RECOVERING for minimum duration
            if time_in_state < MIN_RECOVERY_DURATION:
                return AvoidanceState.RECOVERING

            # NEW: If obstacle re-detected on EITHER side, go back to avoiding
            # This handles cases where we're still alongside a thick obstacle
            if self.has_obstacle and (self.clear_left < 0.5 or self.clear_right < 0.5):
                self.get_logger().warn("[RECOVERY] Obstacle still detected on sides, returning to AVOIDING")
                return AvoidanceState.AVOIDING

            # Check if new obstacle detected ahead during recovery
            if self.has_obstacle and self.obstacle_distance < OBSTACLE_DETECT_RANGE:
                return AvoidanceState.AVOIDING

            # Check recovery completion conditions
            avg_confidence = np.mean(self.sift_confidence_history) if len(self.sift_confidence_history) > 0 else 0.0
            avg_error = np.mean(self.visual_error_history) if len(self.visual_error_history) > 0 else 999.0

            if (avg_confidence > SIFT_CONFIDENCE_THRESHOLD and
                avg_error < SIFT_ERROR_THRESHOLD and
                self.sift_seq != -1):
                return AvoidanceState.NORMAL

            return AvoidanceState.RECOVERING

        # ========== EMERGENCY STOP STATE ==========
        elif current == AvoidanceState.EMERGENCY_STOP:
            # Exit emergency if obstacle moves away
            if not self.has_obstacle or self.obstacle_distance > CRITICAL_DISTANCE + 2.0:
                return AvoidanceState.AVOIDING

            return AvoidanceState.EMERGENCY_STOP

        return current

    def transition_to_state(self, new_state):
        """Handle state transition logic"""
        old_state = self.current_state

        # State entry actions
        if new_state == AvoidanceState.AVOIDING:
            # Remember which side we're avoiding to
            self.last_avoid_side = self.preferred_side
            self.get_logger().info(f"[TRANSITION] NORMAL → AVOIDING | Distance: {self.obstacle_distance:.1f}m | Side: {self.get_side_name(self.last_avoid_side)}")

        elif new_state == AvoidanceState.RECOVERING:
            self.get_logger().info(f"[TRANSITION] AVOIDING → RECOVERING | Obstacle cleared, returning to path...")

        elif new_state == AvoidanceState.NORMAL:
            avg_conf = np.mean(self.sift_confidence_history) if len(self.sift_confidence_history) > 0 else 0.0
            self.get_logger().info(f"[TRANSITION] RECOVERING → NORMAL | Path reacquired. Confidence: {avg_conf:.2f}")

        elif new_state == AvoidanceState.EMERGENCY_STOP:
            self.get_logger().warn(f"[EMERGENCY] Obstacle too close! Distance: {self.obstacle_distance:.1f}m < {CRITICAL_DISTANCE}m")

        self.current_state = new_state
        self.state_enter_time = time.time()

    # ==========================================
    #        COMMAND GENERATION
    # ==========================================

    def generate_command(self):
        """Generate avoidance command based on current state"""

        command = {
            'is_active': 0.0,
            'roll_adjustment': 0.0,
            'speed_multiplier': 1.0,
            'mode': AvoidanceState.NORMAL,
            'look_ahead_multiplier': 1.0
        }

        state = self.current_state

        # ========== NORMAL MODE ==========
        if state == AvoidanceState.NORMAL:
            command['is_active'] = 0.0
            command['roll_adjustment'] = 0.0
            command['speed_multiplier'] = 1.0
            command['mode'] = AvoidanceState.NORMAL
            command['look_ahead_multiplier'] = 1.0

        # ========== AVOIDING MODE ==========
        elif state == AvoidanceState.AVOIDING:
            command['is_active'] = 1.0
            command['mode'] = AvoidanceState.AVOIDING

            # Calculate roll adjustment based on side and urgency
            base_roll = AVOIDANCE_ROLL_GAIN * self.preferred_side

            # Amplify if high urgency
            if self.obstacle_urgency > URGENCY_THRESHOLD:
                base_roll *= 1.5

            # NEW: If BOTH sides are blocked (thick obstacle), amplify MORE
            both_sides_blocked = (self.clear_left < 0.5 and self.clear_right < 0.5)
            if both_sides_blocked:
                base_roll *= 1.3  # Extra 30% for thick obstacles
                self.get_logger().info("[AVOIDING] Thick obstacle detected - amplifying avoidance!")

            command['roll_adjustment'] = base_roll
            command['speed_multiplier'] = AVOIDANCE_SPEED_MULT
            command['look_ahead_multiplier'] = 1.0

        # ========== RECOVERING MODE ==========
        elif state == AvoidanceState.RECOVERING:
            command['is_active'] = 1.0
            command['mode'] = AvoidanceState.RECOVERING

            # NEW: Coast straight for a period before turning back
            # This ensures we're well past thick obstacles before returning to path
            time_in_recovery = time.time() - self.state_enter_time

            if time_in_recovery > RECOVERY_COAST_TIME:
                # After coasting, steer back toward centerline
                recovery_roll = -self.last_avoid_side * RECOVERY_ROLL_GAIN
            else:
                # Coast straight - don't turn back yet
                recovery_roll = 0.0

            command['roll_adjustment'] = recovery_roll
            command['speed_multiplier'] = RECOVERY_SPEED_MULT
            command['look_ahead_multiplier'] = 1.5  # Increase look-ahead for better path reacquisition

        # ========== EMERGENCY STOP ==========
        elif state == AvoidanceState.EMERGENCY_STOP:
            command['is_active'] = 1.0
            command['mode'] = AvoidanceState.EMERGENCY_STOP
            command['roll_adjustment'] = 0.0
            command['speed_multiplier'] = 0.0  # STOP
            command['look_ahead_multiplier'] = 1.0

        return command

    # ==========================================
    #        PUBLISHING
    # ==========================================

    def publish_command(self, command):
        """Publish avoidance command to path follower"""
        msg = Float32MultiArray()
        msg.data = [
            float(command['is_active']),
            float(command['roll_adjustment']),
            float(command['speed_multiplier']),
            float(command['mode']),
            float(command['look_ahead_multiplier'])
        ]

        self.command_pub.publish(msg)

    # ==========================================
    #        LOGGING & UTILITIES
    # ==========================================

    def log_status(self):
        """Periodic status logging"""
        current_time = time.time()

        if current_time - self.last_log_time < LOG_INTERVAL:
            return

        self.last_log_time = current_time

        state_name = self.get_state_name(self.current_state)

        if self.current_state == AvoidanceState.NORMAL:
            if self.has_obstacle:
                self.get_logger().info(f"[{state_name}] Obstacle detected @ {self.obstacle_distance:.1f}m (monitoring)")
            else:
                self.get_logger().info(f"[{state_name}] All clear | SIFT: Seq={int(self.sift_seq)}, Conf={self.sift_confidence:.2f}")

        elif self.current_state == AvoidanceState.AVOIDING:
            side = self.get_side_name(self.preferred_side)
            self.get_logger().info(f"[{state_name}] Avoiding {side} | Dist: {self.obstacle_distance:.1f}m | Urg: {self.obstacle_urgency:.2f} | ClearL:{self.clear_left:.2f} ClearR:{self.clear_right:.2f}")

        elif self.current_state == AvoidanceState.RECOVERING:
            avg_conf = np.mean(self.sift_confidence_history) if len(self.sift_confidence_history) > 0 else 0.0
            avg_err = np.mean(self.visual_error_history) if len(self.visual_error_history) > 0 else 0.0
            self.get_logger().info(f"[{state_name}] Recovery progress | SIFT Conf: {avg_conf:.2f} | VisErr: {avg_err:.1f}px | Target: >{SIFT_CONFIDENCE_THRESHOLD:.2f} & <{SIFT_ERROR_THRESHOLD:.1f}px")

        elif self.current_state == AvoidanceState.EMERGENCY_STOP:
            self.get_logger().warn(f"[{state_name}] STOPPED! Obstacle @ {self.obstacle_distance:.1f}m")

    def get_state_name(self, state):
        """Convert state enum to readable string"""
        names = {
            AvoidanceState.NORMAL: "NORMAL",
            AvoidanceState.AVOIDING: "AVOIDING",
            AvoidanceState.RECOVERING: "RECOVERING",
            AvoidanceState.EMERGENCY_STOP: "EMERGENCY"
        }
        return names.get(state, "UNKNOWN")

    def get_side_name(self, side):
        """Convert side enum to readable string"""
        if side == -1:
            return "LEFT"
        elif side == 1:
            return "RIGHT"
        else:
            return "NONE"


# ==========================================
#        MAIN
# ==========================================

def main(args=None):
    rclpy.init(args=args)

    node = ObstacleAvoidanceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
