#!/usr/bin/env python3
"""Convert ROS 2 recorded episodes to LeRobot v3 dataset format.

Input:  ~/lerobot_data/episode_NNNN/ (from record_episode.py)
Output: LeRobot-compatible dataset (parquet + video)

The output can be used directly with:
    lerobot-train --dataset.repo_id=local:output_dir --policy.type=smolvla

Usage:
    python3 convert_to_lerobot.py \\
        --input ~/lerobot_data \\
        --output ~/lerobot_dataset \\
        --repo-id my_user/openarm_bottle_pickup

Agent-C2 (Vision/VLA) — Phase 3, Task 3.2
"""

import os
import csv
import json
import argparse
from pathlib import Path

import numpy as np

try:
    import torch
    from PIL import Image
    HAS_DEPS = True
except ImportError:
    HAS_DEPS = False

try:
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    HAS_LEROBOT = True
except ImportError:
    HAS_LEROBOT = False


# OpenArm right arm joint names (must match recording)
JOINT_NAMES = [
    'joint1', 'joint2', 'joint3', 'joint4',
    'joint5', 'joint6', 'joint7', 'gripper',
]


def load_episode(episode_dir):
    """Load a single recorded episode.

    Returns:
        dict with keys: task, positions, velocities, timestamps,
                        head_images, wrist_images
    """
    episode_dir = Path(episode_dir)

    # Load metadata
    meta_path = episode_dir / 'metadata.json'
    if not meta_path.exists():
        raise FileNotFoundError(f'No metadata.json in {episode_dir}')
    with open(meta_path) as f:
        meta = json.load(f)

    # Load joints
    csv_path = episode_dir / 'joints.csv'
    timestamps = []
    positions = []
    velocities = []

    with open(csv_path) as f:
        reader = csv.reader(f)
        header = next(reader)
        for row in reader:
            timestamps.append(float(row[0]))
            positions.append([float(x) for x in row[1:9]])   # 8 values
            velocities.append([float(x) for x in row[9:17]])  # 8 values

    # Load head camera images
    head_images = []
    head_dir = episode_dir / 'head_cam'
    if head_dir.exists():
        for img_path in sorted(head_dir.glob('frame_*.png')):
            head_images.append(str(img_path))

    # Load wrist camera images
    wrist_images = []
    wrist_dir = episode_dir / 'wrist_cam'
    if wrist_dir.exists():
        for img_path in sorted(wrist_dir.glob('frame_*.png')):
            wrist_images.append(str(img_path))

    return {
        'task': meta.get('task', 'unknown'),
        'episode_num': meta.get('episode', 0),
        'duration_s': meta.get('duration_s', 0),
        'fps': meta.get('recording_hz', 30),
        'timestamps': timestamps,
        'positions': positions,
        'velocities': velocities,
        'head_images': head_images,
        'wrist_images': wrist_images,
    }


def compute_actions(positions, action_type='absolute'):
    """Compute actions from position sequences.

    Args:
        positions: List of [8] joint position arrays
        action_type: 'absolute' or 'delta'

    Returns:
        List of [8] action arrays
    """
    if action_type == 'absolute':
        # Action = next position (shifted by 1)
        actions = positions[1:] + [positions[-1]]
    elif action_type == 'delta':
        # Action = position change
        actions = []
        for i in range(len(positions) - 1):
            delta = [positions[i+1][j] - positions[i][j]
                     for j in range(len(positions[i]))]
            actions.append(delta)
        actions.append([0.0] * len(positions[0]))  # Last frame: no action
    else:
        raise ValueError(f'Unknown action_type: {action_type}')

    return actions


def convert_to_hf_format(input_dir, output_dir, repo_id=None,
                         action_type='absolute'):
    """Convert all episodes to a LeRobot-compatible HuggingFace dataset.

    Creates a directory structure compatible with LeRobotDataset:
        output_dir/
            data/
                train-00000-of-00001.parquet
            videos/
                head_cam/
                    episode_000000.mp4
                wrist_cam/
                    episode_000000.mp4
            meta/
                info.json
                episodes.jsonl
                tasks.jsonl
    """
    input_dir = Path(input_dir)
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # Find all episodes
    episode_dirs = sorted(input_dir.glob('episode_*'))
    if not episode_dirs:
        print(f'No episodes found in {input_dir}')
        return

    print(f'Found {len(episode_dirs)} episodes in {input_dir}')

    # Collect all data
    all_rows = []
    episode_lengths = []
    tasks = set()

    for ep_idx, ep_dir in enumerate(episode_dirs):
        print(f'  Processing {ep_dir.name}...')
        try:
            ep = load_episode(ep_dir)
        except Exception as e:
            print(f'  ERROR: {e}, skipping')
            continue

        tasks.add(ep['task'])
        actions = compute_actions(ep['positions'], action_type)

        for frame_idx in range(len(ep['positions'])):
            row = {
                'episode_index': ep_idx,
                'frame_index': frame_idx,
                'timestamp': ep['timestamps'][frame_idx] - ep['timestamps'][0],
                'task': ep['task'],
            }

            # State: current joint positions
            for j, name in enumerate(JOINT_NAMES):
                row[f'observation.state.{name}'] = ep['positions'][frame_idx][j]

            # Action: target joint positions (or deltas)
            for j, name in enumerate(JOINT_NAMES):
                row[f'action.{name}'] = actions[frame_idx][j]

            # Image paths (relative)
            if frame_idx < len(ep['head_images']):
                row['observation.images.head_cam'] = ep['head_images'][frame_idx]
            if frame_idx < len(ep['wrist_images']):
                row['observation.images.wrist_cam'] = ep['wrist_images'][frame_idx]

            all_rows.append(row)

        episode_lengths.append(len(ep['positions']))
        print(f'    → {len(ep["positions"])} frames, {ep["duration_s"]:.1f}s')

    if not all_rows:
        print('No data to convert!')
        return

    # Save as parquet (tabular data)
    try:
        import pandas as pd
        df = pd.DataFrame(all_rows)
        data_dir = output_dir / 'data'
        data_dir.mkdir(exist_ok=True)
        parquet_path = data_dir / 'train-00000-of-00001.parquet'
        df.to_parquet(parquet_path, index=False)
        print(f'Saved {len(df)} rows to {parquet_path}')
    except ImportError:
        # Fallback: save as JSON lines
        data_dir = output_dir / 'data'
        data_dir.mkdir(exist_ok=True)
        jsonl_path = data_dir / 'train.jsonl'
        with open(jsonl_path, 'w') as f:
            for row in all_rows:
                f.write(json.dumps(row) + '\n')
        print(f'Saved {len(all_rows)} rows to {jsonl_path} (pandas not available)')

    # Save metadata
    meta_dir = output_dir / 'meta'
    meta_dir.mkdir(exist_ok=True)

    # info.json
    info = {
        'codebase_version': '3.0',
        'robot_type': 'openarm_follower',
        'total_episodes': len(episode_lengths),
        'total_frames': sum(episode_lengths),
        'fps': 30,
        'action_type': action_type,
        'features': {
            'observation.state': {
                'dtype': 'float32',
                'shape': [8],
                'names': JOINT_NAMES,
            },
            'action': {
                'dtype': 'float32',
                'shape': [8],
                'names': JOINT_NAMES,
            },
            'observation.images.head_cam': {
                'dtype': 'image',
                'shape': [480, 640, 3],
            },
            'observation.images.wrist_cam': {
                'dtype': 'image',
                'shape': [480, 640, 3],
            },
        },
    }
    if repo_id:
        info['repo_id'] = repo_id

    with open(meta_dir / 'info.json', 'w') as f:
        json.dump(info, f, indent=2)

    # episodes.jsonl
    with open(meta_dir / 'episodes.jsonl', 'w') as f:
        cum_frames = 0
        for i, length in enumerate(episode_lengths):
            ep_meta = {
                'episode_index': i,
                'length': length,
                'from': cum_frames,
                'to': cum_frames + length,
            }
            f.write(json.dumps(ep_meta) + '\n')
            cum_frames += length

    # tasks.jsonl
    with open(meta_dir / 'tasks.jsonl', 'w') as f:
        for i, task in enumerate(sorted(tasks)):
            f.write(json.dumps({'task_index': i, 'task': task}) + '\n')

    print(f'\n✅ Conversion complete!')
    print(f'   Output: {output_dir}')
    print(f'   Episodes: {len(episode_lengths)}')
    print(f'   Total frames: {sum(episode_lengths)}')
    print(f'   Tasks: {tasks}')
    print(f'\n   To train SmolVLA:')
    print(f'   lerobot-train \\')
    print(f'     --policy.path=lerobot/smolvla_base \\')
    print(f'     --dataset.repo_id={repo_id or "local:" + str(output_dir)} \\')
    print(f'     --batch_size=64 --steps=20000')


def main():
    parser = argparse.ArgumentParser(
        description='Convert ROS 2 episodes to LeRobot dataset format')
    parser.add_argument('--input', default=os.path.expanduser('~/lerobot_data'),
                        help='Input directory with episode_* folders')
    parser.add_argument('--output', default=os.path.expanduser('~/lerobot_dataset'),
                        help='Output directory for LeRobot dataset')
    parser.add_argument('--repo-id', default=None,
                        help='HuggingFace repo ID (e.g., user/openarm_bottle)')
    parser.add_argument('--action-type', default='absolute',
                        choices=['absolute', 'delta'],
                        help='Action representation type')
    args = parser.parse_args()

    convert_to_hf_format(
        input_dir=args.input,
        output_dir=args.output,
        repo_id=args.repo_id,
        action_type=args.action_type,
    )


if __name__ == '__main__':
    main()
