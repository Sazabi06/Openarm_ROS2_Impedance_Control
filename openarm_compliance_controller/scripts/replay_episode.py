#!/usr/bin/env python3
"""Replay a recorded episode to verify data quality.

Sends recorded joint trajectories back to the robot via JTC action server.
Use this to visually verify recordings BEFORE spending hours training.

Usage:
    python3 replay_episode.py --episode 0 [--input ~/lerobot_data] [--speed 1.0]

Agent-C2 (Vision/VLA) — Phase 3, Task 3.2
"""

import os
import csv
import json
import time
import argparse
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import String


# Right arm joint names (matching controller config)
RIGHT_ARM_JOINTS = [
    'openarm_right_joint1',
    'openarm_right_joint2',
    'openarm_right_joint3',
    'openarm_right_joint4',
    'openarm_right_joint5',
    'openarm_right_joint6',
    'openarm_right_joint7',
]


class EpisodeReplayer(Node):
    """Replay a recorded episode on the robot."""

    def __init__(self, episode_dir, speed=1.0, side='right'):
        super().__init__('episode_replayer')
        self.episode_dir = Path(episode_dir)
        self.speed = speed
        self.side = side

        # Phase publisher (set to transit for stiff tracking)
        self.phase_pub = self.create_publisher(String, '/impedance_phase', 10)

        # JTC action client
        self.jtc_client = ActionClient(
            self,
            FollowJointTrajectory,
            f'/{side}_joint_trajectory_controller/follow_joint_trajectory'
        )

        # Gripper action client
        self.gripper_client = ActionClient(
            self,
            GripperCommand,
            f'/{side}_gripper_controller/gripper_cmd'
        )

        self.get_logger().info(f'Loading episode from: {self.episode_dir}')

    def load_episode(self):
        """Load joint data from CSV."""
        csv_path = self.episode_dir / 'joints.csv'
        if not csv_path.exists():
            self.get_logger().error(f'No joints.csv found in {self.episode_dir}')
            return None

        # Load metadata
        meta_path = self.episode_dir / 'metadata.json'
        if meta_path.exists():
            with open(meta_path) as f:
                meta = json.load(f)
            self.get_logger().info(f'Task: {meta.get("task", "unknown")}')
            self.get_logger().info(
                f'Duration: {meta.get("duration_s", 0):.1f}s, '
                f'{meta.get("num_frames", 0)} frames')

        # Parse CSV
        frames = []
        with open(csv_path) as f:
            reader = csv.reader(f)
            header = next(reader)
            for row in reader:
                timestamp = float(row[0])
                # 8 positions (7 joints + gripper)
                positions = [float(x) for x in row[1:9]]
                frames.append({
                    'timestamp': timestamp,
                    'positions': positions[:7],  # arm joints for JTC
                    'gripper': positions[7] if len(positions) > 7 else 0.0,
                })

        self.get_logger().info(f'Loaded {len(frames)} frames')
        return frames

    def replay(self):
        """Send trajectory to JTC."""
        frames = self.load_episode()
        if not frames or len(frames) < 2:
            self.get_logger().error('Not enough frames to replay')
            return

        # Wait for JTC
        self.get_logger().info('Waiting for JTC action server...')
        if not self.jtc_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('JTC action server not available!')
            return

        # Set transit mode for stiff tracking
        phase_msg = String()
        phase_msg.data = 'transit'
        self.phase_pub.publish(phase_msg)
        time.sleep(0.5)

        # Build trajectory
        t0 = frames[0]['timestamp']
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = RIGHT_ARM_JOINTS

        # Subsample to ~10 Hz for smooth trajectory (every 3rd frame at 30Hz)
        step = max(1, int(3 / self.speed))
        for i in range(0, len(frames), step):
            frame = frames[i]
            dt = (frame['timestamp'] - t0) / self.speed

            point = JointTrajectoryPoint()
            point.positions = frame['positions']
            point.time_from_start = Duration(
                sec=int(dt), nanosec=int((dt % 1) * 1e9))
            goal.trajectory.points.append(point)

        total_time = (frames[-1]['timestamp'] - t0) / self.speed
        self.get_logger().info(
            f'Replaying {len(goal.trajectory.points)} waypoints over '
            f'{total_time:.1f}s (speed={self.speed}x)')

        # Send goal
        future = self.jtc_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is None:
            self.get_logger().error('Failed to send goal')
            return

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by JTC')
            return

        self.get_logger().info('🔄 Replaying... (Ctrl+C to abort)')

        # Replay gripper in sync — send gripper commands at ~3 Hz
        gripper_step = max(1, int(10 / self.speed))  # every 10th frame at 30Hz
        gripper_frames = [(frames[i]['timestamp'] - t0, frames[i]['gripper'])
                          for i in range(0, len(frames), gripper_step)]

        # Send gripper commands in a background thread-like loop
        has_gripper = self.gripper_client.wait_for_server(timeout_sec=2.0)
        if has_gripper:
            self.get_logger().info('Gripper replay enabled')
            replay_start = time.time()
            grip_idx = 0
            last_grip_pos = None

            while grip_idx < len(gripper_frames):
                elapsed = (time.time() - replay_start)
                target_t, grip_pos = gripper_frames[grip_idx]

                if elapsed >= target_t / self.speed:
                    # Only send if position changed significantly (>1mm)
                    if last_grip_pos is None or abs(grip_pos - last_grip_pos) > 0.001:
                        grip_goal = GripperCommand.Goal()
                        grip_goal.command.position = grip_pos
                        grip_goal.command.max_effort = 10.0
                        self.gripper_client.send_goal_async(grip_goal)
                        last_grip_pos = grip_pos
                    grip_idx += 1
                else:
                    time.sleep(0.01)
        else:
            self.get_logger().warn('Gripper server not available — skipping gripper replay')

        # Wait for arm trajectory completion
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=total_time + 10.0)

        self.get_logger().info('✅ Replay complete!')


def main():
    parser = argparse.ArgumentParser(
        description='Replay a recorded episode for verification')
    parser.add_argument('--episode', type=int, required=True,
                        help='Episode number to replay')
    parser.add_argument('--input', default=os.path.expanduser('~/lerobot_data'),
                        help='Input directory (default: ~/lerobot_data)')
    parser.add_argument('--speed', type=float, default=1.0,
                        help='Playback speed multiplier (default: 1.0)')
    parser.add_argument('--side', default='right', choices=['left', 'right'])
    args = parser.parse_args()

    episode_dir = Path(args.input) / f'episode_{args.episode:04d}'
    if not episode_dir.exists():
        print(f'Episode directory not found: {episode_dir}')
        return

    rclpy.init()
    node = EpisodeReplayer(
        episode_dir=episode_dir,
        speed=args.speed,
        side=args.side,
    )

    try:
        node.replay()
    except KeyboardInterrupt:
        node.get_logger().info('Replay aborted')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
