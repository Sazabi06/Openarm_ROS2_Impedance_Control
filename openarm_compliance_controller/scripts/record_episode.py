#!/usr/bin/env python3
"""Record drag-to-teach episodes for VLA training.

Records synchronized multimodal data during drag-to-teach:
  - Joint states (7 arm + gripper) at 30 Hz
  - Head camera (D435i) at 30 Hz
  - Right wrist camera (D405) at 30 Hz (optional)

Episodes are saved to ~/lerobot_data/episode_{NNN}/ as:
  - joints.csv  — timestamp, positions, velocities, efforts
  - head_cam/   — PNG frames (frame_NNNNN.png)
  - wrist_cam/  — PNG frames (frame_NNNNN.png)
  - metadata.json — task description, timestamps, camera info

Usage:
    python3 record_episode.py --task "Pick up the bottle" --num-episodes 5

Controls:
    s  — start recording current episode
    e  — end current episode
    d  — discard current episode
    q  — quit

Agent-C2 (Vision/VLA) — Phase 3, Task 3.2
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


# Joint names for right arm (7 DOF + gripper)
RIGHT_ARM_JOINTS = [
    'openarm_right_joint1',
    'openarm_right_joint2',
    'openarm_right_joint3',
    'openarm_right_joint4',
    'openarm_right_joint5',
    'openarm_right_joint6',
    'openarm_right_joint7',
]
GRIPPER_JOINT = 'openarm_right_finger_joint1'


class EpisodeRecorder(Node):
    """Records synchronized joint + camera data for VLA training."""

    RECORDING_HZ = 30  # Target recording frequency

    def __init__(self, task_description, output_dir, side='right',
                 use_wrist_cam=True):
        super().__init__('episode_recorder')

        self.task = task_description
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.side = side
        self.use_wrist_cam = use_wrist_cam

        # State
        self.recording = False
        self.episode_num = self._next_episode_num()
        self.episode_data = []  # List of {timestamp, joints, ...}
        self.head_frame_count = 0
        self.wrist_frame_count = 0
        self.frame_count = 0
        self._lock = threading.Lock()

        # Background image writer (prevents RAM buildup)
        self._write_queue = queue.Queue()
        self._writer_thread = threading.Thread(
            target=self._disk_writer_loop, daemon=True)
        self._writer_thread.start()
        self._episode_dir = None  # Set when recording starts

        # CV bridge for image conversion
        self.bridge = CvBridge() if HAS_CV else None

        # Latest images (updated by subscribers)
        self.latest_head_img = None
        self.latest_wrist_img = None

        # Subscribers
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self._joint_cb, 10)

        self.head_cam_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self._head_cam_cb, 10)

        if use_wrist_cam:
            self.wrist_cam_sub = self.create_subscription(
                Image, f'/{side}_wrist_camera/color/image_raw',
                self._wrist_cam_cb, 10)

        # Recording timer (30 Hz)
        self.record_timer = self.create_timer(
            1.0 / self.RECORDING_HZ, self._record_tick)

        self.get_logger().info(
            '╔══════════════════════════════════════════════╗')
        self.get_logger().info(
            '║       Episode Recorder Ready                 ║')
        self.get_logger().info(
            f'║  Task: {task_description[:38]:<38s} ║')
        self.get_logger().info(
            f'║  Next episode: #{self.episode_num:<29d} ║')
        self.get_logger().info(
            f'║  Cameras: head' +
            (' + wrist' if use_wrist_cam else ' only') +
            ' ' * (23 - (8 if use_wrist_cam else 5)) + '║')
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
        """Cache latest joint state."""
        with self._lock:
            self._latest_joint_msg = msg

    def _head_cam_cb(self, msg):
        """Cache latest head camera image."""
        if self.bridge:
            try:
                self.latest_head_img = self.bridge.imgmsg_to_cv2(
                    msg, desired_encoding='bgr8')
            except Exception:
                pass

    def _wrist_cam_cb(self, msg):
        """Cache latest wrist camera image."""
        if self.bridge:
            try:
                self.latest_wrist_img = self.bridge.imgmsg_to_cv2(
                    msg, desired_encoding='bgr8')
            except Exception:
                pass

    def _record_tick(self):
        """Called at 30 Hz — record one synchronized frame if recording."""
        if not self.recording:
            return

        with self._lock:
            joint_msg = getattr(self, '_latest_joint_msg', None)

        if joint_msg is None:
            return

        timestamp = time.time()

        # Extract arm joint positions in order
        joint_positions = []
        joint_velocities = []
        joint_efforts = []

        for jname in RIGHT_ARM_JOINTS + [GRIPPER_JOINT]:
            if jname in joint_msg.name:
                idx = joint_msg.name.index(jname)
                joint_positions.append(joint_msg.position[idx])
                joint_velocities.append(
                    joint_msg.velocity[idx] if idx < len(joint_msg.velocity) else 0.0)
                joint_efforts.append(
                    joint_msg.effort[idx] if idx < len(joint_msg.effort) else 0.0)
            else:
                joint_positions.append(0.0)
                joint_velocities.append(0.0)
                joint_efforts.append(0.0)

        # Store data
        self.episode_data.append({
            'timestamp': timestamp,
            'positions': joint_positions,
            'velocities': joint_velocities,
            'efforts': joint_efforts,
        })

        # Write images to disk incrementally via background thread
        if self.latest_head_img is not None:
            self._write_queue.put((
                self._episode_dir / 'head_cam' / f'frame_{self.head_frame_count:05d}.png',
                self.latest_head_img.copy()
            ))
            self.head_frame_count += 1

        if self.use_wrist_cam and self.latest_wrist_img is not None:
            self._write_queue.put((
                self._episode_dir / 'wrist_cam' / f'frame_{self.wrist_frame_count:05d}.png',
                self.latest_wrist_img.copy()
            ))
            self.wrist_frame_count += 1

        self.frame_count += 1
        if self.frame_count % 30 == 0:  # Log every second
            self.get_logger().info(
                f'  Recording... {self.frame_count} frames '
                f'({self.frame_count / self.RECORDING_HZ:.1f}s)')

    def start_recording(self):
        """Start recording a new episode."""
        if self.recording:
            self.get_logger().warn('Already recording! End current episode first.')
            return

        self.recording = True
        self.episode_data = []
        self.head_frame_count = 0
        self.wrist_frame_count = 0
        self.frame_count = 0
        self.record_start = time.time()

        # Create episode directory and camera subdirs immediately
        self._episode_dir = self.output_dir / f'episode_{self.episode_num:04d}'
        self._episode_dir.mkdir(parents=True, exist_ok=True)
        (self._episode_dir / 'head_cam').mkdir(exist_ok=True)
        if self.use_wrist_cam:
            (self._episode_dir / 'wrist_cam').mkdir(exist_ok=True)

        self.get_logger().info(
            f'🔴 Recording episode #{self.episode_num}...')

    def end_recording(self):
        """End and save current episode."""
        if not self.recording:
            self.get_logger().warn('Not recording!')
            return

        self.recording = False
        duration = time.time() - self.record_start

        if self.frame_count < 10:
            self.get_logger().warn(
                f'Episode too short ({self.frame_count} frames). Discarding.')
            return

        # Save episode
        episode_dir = self._episode_dir

        # Wait for background writer to finish pending frames
        self._write_queue.join()

        # Save joints CSV
        csv_path = episode_dir / 'joints.csv'
        with open(csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            header = ['timestamp'] + \
                     [f'pos_{i}' for i in range(8)] + \
                     [f'vel_{i}' for i in range(8)] + \
                     [f'eff_{i}' for i in range(8)]
            writer.writerow(header)
            for frame in self.episode_data:
                row = [frame['timestamp']] + \
                      frame['positions'] + \
                      frame['velocities'] + \
                      frame['efforts']
                writer.writerow(row)

        # Head and wrist frames already saved to disk incrementally

        # Save metadata
        metadata = {
            'task': self.task,
            'episode': self.episode_num,
            'duration_s': duration,
            'num_frames': self.frame_count,
            'recording_hz': self.RECORDING_HZ,
            'num_head_frames': self.head_frame_count,
            'num_wrist_frames': self.wrist_frame_count,
            'joint_names': RIGHT_ARM_JOINTS + [GRIPPER_JOINT],
            'start_time': datetime.fromtimestamp(
                self.record_start).isoformat(),
            'end_time': datetime.now().isoformat(),
        }
        with open(episode_dir / 'metadata.json', 'w') as f:
            json.dump(metadata, f, indent=2)

        self.get_logger().info(
            f'✅ Episode #{self.episode_num} saved: '
            f'{self.frame_count} frames, {duration:.1f}s, '
            f'{self.head_frame_count} head imgs, '
            f'{self.wrist_frame_count} wrist imgs')
        self.get_logger().info(f'   → {episode_dir}')

        self.episode_num += 1

    def discard_recording(self):
        """Discard current recording."""
        if not self.recording:
            self.get_logger().warn('Not recording!')
            return

        self.recording = False
        self.get_logger().info(
            f'🗑️  Episode #{self.episode_num} discarded '
            f'({self.frame_count} frames)')

    def run_keyboard_loop(self):
        """Handle keyboard input."""
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
        description='Record drag-to-teach episodes for VLA training')
    parser.add_argument('--task', required=True,
                        help='Task description (e.g., "Pick up the bottle")')
    parser.add_argument('--output', default=os.path.expanduser('~/lerobot_data'),
                        help='Output directory (default: ~/lerobot_data)')
    parser.add_argument('--side', default='right', choices=['left', 'right'],
                        help='Which arm')
    parser.add_argument('--no-wrist-cam', action='store_true',
                        help='Disable wrist camera recording')
    parser.add_argument('--num-episodes', type=int, default=0,
                        help='Auto-quit after N episodes (0=unlimited)')
    args = parser.parse_args()

    rclpy.init()
    node = EpisodeRecorder(
        task_description=args.task,
        output_dir=args.output,
        side=args.side,
        use_wrist_cam=not args.no_wrist_cam,
    )

    # Spin ROS in background
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Keyboard in main thread
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
