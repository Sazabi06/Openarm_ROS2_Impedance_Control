#!/usr/bin/env python3
"""Record bimanual drag-to-teach episodes for π0.5 training.

Records synchronized multimodal data during bimanual drag-to-teach:
  - Joint states: 16D [left_J1..J7, left_grip, right_J1..J7, right_grip] at 30 Hz
  - Head camera (D435I, serial 243122070766) at 30 Hz
  - Left wrist camera (D405, serial 323622271581) at 30 Hz
  - Right wrist camera (D405, serial 335122273029) at 30 Hz

Output format: LeRobot v3 compatible for OpenPI π0.5 fine-tuning.

Usage:
    # With both arms in teach mode:
    python3 record_bimanual_episode.py --task "Pick up the bottle"

Controls:
    s  — start recording current episode
    e  — end current episode (save)
    d  — discard current episode
    q  — quit
"""

import os
import sys
import csv
import json
import time
import argparse
import threading
import queue
from pathlib import Path
from datetime import datetime

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image

try:
    from cv_bridge import CvBridge
    import cv2
    HAS_CV = True
except ImportError:
    HAS_CV = False
    print("WARNING: cv_bridge/opencv not available. Images will not be saved.")

# Non-blocking keyboard
try:
    import termios
    import tty

    def _getch():
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            return sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
except ImportError:
    def _getch():
        return input("key> ")[0] if input("key> ") else ""


# ============================================================================
# Bimanual Joint Layout — matches OpenPI pi05_openarm config
# ============================================================================
# 16D: [left_J1..J7, left_grip, right_J1..J7, right_grip]
LEFT_ARM_JOINTS = [
    'openarm_left_joint1',
    'openarm_left_joint2',
    'openarm_left_joint3',
    'openarm_left_joint4',
    'openarm_left_joint5',
    'openarm_left_joint6',
    'openarm_left_joint7',
]
LEFT_GRIPPER = 'openarm_left_finger_joint1'

RIGHT_ARM_JOINTS = [
    'openarm_right_joint1',
    'openarm_right_joint2',
    'openarm_right_joint3',
    'openarm_right_joint4',
    'openarm_right_joint5',
    'openarm_right_joint6',
    'openarm_right_joint7',
]
RIGHT_GRIPPER = 'openarm_right_finger_joint1'

ALL_JOINTS_ORDERED = LEFT_ARM_JOINTS + [LEFT_GRIPPER] + RIGHT_ARM_JOINTS + [RIGHT_GRIPPER]

JOINT_NAMES_16D = [
    'left_joint1', 'left_joint2', 'left_joint3', 'left_joint4',
    'left_joint5', 'left_joint6', 'left_joint7', 'left_gripper',
    'right_joint1', 'right_joint2', 'right_joint3', 'right_joint4',
    'right_joint5', 'right_joint6', 'right_joint7', 'right_gripper',
]


class BimanualEpisodeRecorder(Node):
    """Records synchronized bimanual joint + 3-camera data for π0.5 training."""

    RECORDING_HZ = 30  # Target recording frequency

    def __init__(self, task_description, output_dir):
        super().__init__('bimanual_episode_recorder')

        self.task = task_description
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

        # State
        self.recording = False
        self.episode_num = self._next_episode_num()
        self.episode_data = []
        self.frame_counts = {'head': 0, 'wrist_left': 0, 'wrist_right': 0}
        self.frame_count = 0
        self._lock = threading.Lock()
        self._latest_joint_msg = None

        # Background image writer
        self._write_queue = queue.Queue()
        self._writer_thread = threading.Thread(
            target=self._disk_writer_loop, daemon=True)
        self._writer_thread.start()
        self._episode_dir = None

        # CV bridge
        self.bridge = CvBridge() if HAS_CV else None

        # Latest images
        self.latest_head_img = None
        self.latest_wrist_left_img = None
        self.latest_wrist_right_img = None

        # === Subscribers ===
        # Joint states (bimanual — both arms publish to /joint_states)
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self._joint_cb, 10)

        # Head camera (D435I)
        self.head_cam_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self._head_cam_cb, 10)

        # Left wrist camera (D405)
        self.wrist_left_sub = self.create_subscription(
            Image, '/left_wrist_camera/color/image_raw',
            self._wrist_left_cb, 10)

        # Right wrist camera (D405)
        self.wrist_right_sub = self.create_subscription(
            Image, '/right_wrist_camera/color/image_raw',
            self._wrist_right_cb, 10)

        # Recording timer (30 Hz)
        self.record_timer = self.create_timer(
            1.0 / self.RECORDING_HZ, self._record_tick)

        self.get_logger().info(
            '╔══════════════════════════════════════════════╗')
        self.get_logger().info(
            '║    Bimanual Episode Recorder Ready           ║')
        self.get_logger().info(
            f'║  Task: {task_description[:38]:<38s} ║')
        self.get_logger().info(
            f'║  Next episode: #{self.episode_num:<29d} ║')
        self.get_logger().info(
            '║  Cameras: head + wrist_left + wrist_right    ║')
        self.get_logger().info(
            '║  State: 16D bimanual [L7+G, R7+G]            ║')
        self.get_logger().info(
            '╠══════════════════════════════════════════════╣')
        self.get_logger().info(
            '║  s = start recording                         ║')
        self.get_logger().info(
            '║  e = end episode (save)                      ║')
        self.get_logger().info(
            '║  d = discard current episode                 ║')
        self.get_logger().info(
            '║  q = quit                                    ║')
        self.get_logger().info(
            '╚══════════════════════════════════════════════╝')

    def _disk_writer_loop(self):
        """Background thread: write images to disk from queue."""
        while True:
            try:
                path, img = self._write_queue.get(timeout=1.0)
                if HAS_CV:
                    cv2.imwrite(str(path), img)
                self._write_queue.task_done()
            except queue.Empty:
                continue

    def _next_episode_num(self):
        """Find the next available episode number."""
        existing = list(self.output_dir.glob('episode_*'))
        if not existing:
            return 0
        nums = []
        for d in existing:
            try:
                nums.append(int(d.name.split('_')[1]))
            except (ValueError, IndexError):
                pass
        return max(nums) + 1 if nums else 0

    def _joint_cb(self, msg):
        with self._lock:
            self._latest_joint_msg = msg

    def _head_cam_cb(self, msg):
        if self.bridge:
            try:
                self.latest_head_img = self.bridge.imgmsg_to_cv2(
                    msg, desired_encoding='bgr8')
            except Exception:
                pass

    def _wrist_left_cb(self, msg):
        if self.bridge:
            try:
                self.latest_wrist_left_img = self.bridge.imgmsg_to_cv2(
                    msg, desired_encoding='bgr8')
            except Exception:
                pass

    def _wrist_right_cb(self, msg):
        if self.bridge:
            try:
                self.latest_wrist_right_img = self.bridge.imgmsg_to_cv2(
                    msg, desired_encoding='bgr8')
            except Exception:
                pass

    def _record_tick(self):
        """Called at 30 Hz — record one synchronized frame if recording."""
        if not self.recording:
            return

        with self._lock:
            joint_msg = self._latest_joint_msg

        if joint_msg is None:
            return

        timestamp = time.time()

        # Extract 16D joint positions in order: [L_J1..J7, L_grip, R_J1..J7, R_grip]
        positions_16d = []
        velocities_16d = []
        efforts_16d = []

        for jname in ALL_JOINTS_ORDERED:
            if jname in joint_msg.name:
                idx = joint_msg.name.index(jname)
                positions_16d.append(joint_msg.position[idx])
                velocities_16d.append(
                    joint_msg.velocity[idx] if idx < len(joint_msg.velocity) else 0.0)
                efforts_16d.append(
                    joint_msg.effort[idx] if idx < len(joint_msg.effort) else 0.0)
            else:
                positions_16d.append(0.0)
                velocities_16d.append(0.0)
                efforts_16d.append(0.0)

        self.episode_data.append({
            'timestamp': timestamp,
            'positions': positions_16d,
            'velocities': velocities_16d,
            'efforts': efforts_16d,
        })

        # Write images to disk incrementally
        if self.latest_head_img is not None:
            self._write_queue.put((
                self._episode_dir / 'head_cam' / f'frame_{self.frame_counts["head"]:05d}.png',
                self.latest_head_img.copy()
            ))
            self.frame_counts['head'] += 1

        if self.latest_wrist_left_img is not None:
            self._write_queue.put((
                self._episode_dir / 'wrist_left_cam' / f'frame_{self.frame_counts["wrist_left"]:05d}.png',
                self.latest_wrist_left_img.copy()
            ))
            self.frame_counts['wrist_left'] += 1

        if self.latest_wrist_right_img is not None:
            self._write_queue.put((
                self._episode_dir / 'wrist_right_cam' / f'frame_{self.frame_counts["wrist_right"]:05d}.png',
                self.latest_wrist_right_img.copy()
            ))
            self.frame_counts['wrist_right'] += 1

        self.frame_count += 1
        if self.frame_count % 30 == 0:
            n_missing = sum(1 for k in ['head', 'wrist_left', 'wrist_right']
                           if self.frame_counts[k] == 0)
            cam_status = f"cams: {self.frame_counts['head']}h/{self.frame_counts['wrist_left']}l/{self.frame_counts['wrist_right']}r"
            self.get_logger().info(
                f'  Recording... {self.frame_count} frames '
                f'({self.frame_count / self.RECORDING_HZ:.1f}s) {cam_status}')

    def start_recording(self):
        if self.recording:
            self.get_logger().warn('Already recording!')
            return

        self.recording = True
        self.episode_data = []
        self.frame_counts = {'head': 0, 'wrist_left': 0, 'wrist_right': 0}
        self.frame_count = 0
        self.record_start = time.time()

        self._episode_dir = self.output_dir / f'episode_{self.episode_num:04d}'
        self._episode_dir.mkdir(parents=True, exist_ok=True)
        (self._episode_dir / 'head_cam').mkdir(exist_ok=True)
        (self._episode_dir / 'wrist_left_cam').mkdir(exist_ok=True)
        (self._episode_dir / 'wrist_right_cam').mkdir(exist_ok=True)

        # Status check
        cam_ok = []
        if self.latest_head_img is not None:
            cam_ok.append('head')
        if self.latest_wrist_left_img is not None:
            cam_ok.append('wrist_left')
        if self.latest_wrist_right_img is not None:
            cam_ok.append('wrist_right')

        self.get_logger().info(
            f'🔴 Recording episode #{self.episode_num}...')
        self.get_logger().info(
            f'   Active cameras: {cam_ok if cam_ok else "NONE - check topics!"}')
        if self._latest_joint_msg:
            n = len(self._latest_joint_msg.name)
            self.get_logger().info(f'   Joint states: {n} joints visible')

    def end_recording(self):
        if not self.recording:
            self.get_logger().warn('Not recording!')
            return

        self.recording = False
        duration = time.time() - self.record_start

        if self.frame_count < 10:
            self.get_logger().warn(
                f'Episode too short ({self.frame_count} frames). Discarding.')
            return

        episode_dir = self._episode_dir
        self._write_queue.join()  # Wait for background writer

        # Save joints CSV (16D)
        csv_path = episode_dir / 'joints.csv'
        with open(csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            header = ['timestamp'] + \
                     [f'pos_{i}' for i in range(16)] + \
                     [f'vel_{i}' for i in range(16)] + \
                     [f'eff_{i}' for i in range(16)]
            writer.writerow(header)
            for frame in self.episode_data:
                row = [frame['timestamp']] + \
                      frame['positions'] + \
                      frame['velocities'] + \
                      frame['efforts']
                writer.writerow(row)

        # Save metadata
        metadata = {
            'task': self.task,
            'episode': self.episode_num,
            'duration_s': duration,
            'num_frames': self.frame_count,
            'recording_hz': self.RECORDING_HZ,
            'camera_frames': self.frame_counts,
            'joint_names_16d': JOINT_NAMES_16D,
            'joint_layout': '[left_J1..J7, left_grip, right_J1..J7, right_grip]',
            'cameras': {
                'head': 'D435I (243122070766)',
                'wrist_left': 'D405 (323622271581)',
                'wrist_right': 'D405 (335122273029)',
            },
            'start_time': datetime.fromtimestamp(self.record_start).isoformat(),
            'end_time': datetime.now().isoformat(),
        }
        with open(episode_dir / 'metadata.json', 'w') as f:
            json.dump(metadata, f, indent=2)

        self.get_logger().info(
            f'✅ Episode #{self.episode_num} saved: '
            f'{self.frame_count} frames, {duration:.1f}s')
        self.get_logger().info(
            f'   Cameras: {self.frame_counts}')
        self.get_logger().info(f'   → {episode_dir}')

        self.episode_num += 1

    def discard_recording(self):
        if not self.recording:
            self.get_logger().warn('Not recording!')
            return
        self.recording = False
        self.get_logger().info(
            f'🗑️  Episode #{self.episode_num} discarded '
            f'({self.frame_count} frames)')

    def run_keyboard_loop(self):
        while rclpy.ok():
            try:
                key = _getch()
            except (EOFError, KeyboardInterrupt):
                break

            if key == 's':
                self.start_recording()
            elif key == 'e':
                self.end_recording()
            elif key == 'd':
                self.discard_recording()
            elif key == 'q':
                if self.recording:
                    self.end_recording()
                self.get_logger().info('Quitting recorder.')
                rclpy.shutdown()
                break


def main():
    parser = argparse.ArgumentParser(
        description='Record bimanual drag-to-teach episodes for π0.5 training')
    parser.add_argument('--task', required=True,
                        help='Task description (e.g., "Pick up the bottle")')
    parser.add_argument('--output', default=os.path.expanduser('~/lerobot_bimanual_data'),
                        help='Output directory (default: ~/lerobot_bimanual_data)')
    args = parser.parse_args()

    rclpy.init()
    node = BimanualEpisodeRecorder(
        task_description=args.task,
        output_dir=args.output,
    )

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.run_keyboard_loop()
    except KeyboardInterrupt:
        if node.recording:
            node.end_recording()
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
