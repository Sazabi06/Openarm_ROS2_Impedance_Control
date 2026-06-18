#!/usr/bin/env python3
"""Convert raw bimanual episodes to LeRobot v2.1 format for π0.5 training.

Takes the raw episodes from record_bimanual_episode.py and creates a proper
LeRobot dataset with per-episode parquets, metadata, and correct feature layout.

Output format matches the community pi05_openarm config:
  - State/Action: 16D [left_J1..J7, left_grip, right_J1..J7, right_grip]
  - Cameras: head, wrist_left, wrist_right (stored as PNG bytes in parquet)

Usage:
    python3 convert_bimanual_to_lerobot.py \
        --input ~/lerobot_bimanual_data \
        --output ~/lerobot_bimanual_dataset \
        --task "Pick up the bottle"
"""

import argparse
import json
import csv
import os
import sys
from pathlib import Path

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq


def load_episode_raw(episode_dir):
    """Load a raw recorded episode (joints CSV + images)."""
    episode_dir = Path(episode_dir)

    # Load metadata
    with open(episode_dir / 'metadata.json') as f:
        metadata = json.load(f)

    # Load joints CSV
    frames = []
    with open(episode_dir / 'joints.csv') as f:
        reader = csv.DictReader(f)
        for row in reader:
            positions = [float(row[f'pos_{i}']) for i in range(16)]
            frames.append({'positions': positions})

    # Load images
    head_dir = episode_dir / 'head_cam'
    wrist_left_dir = episode_dir / 'wrist_left_cam'
    wrist_right_dir = episode_dir / 'wrist_right_cam'

    head_imgs = sorted(head_dir.glob('frame_*.png')) if head_dir.exists() else []
    wrist_left_imgs = sorted(wrist_left_dir.glob('frame_*.png')) if wrist_left_dir.exists() else []
    wrist_right_imgs = sorted(wrist_right_dir.glob('frame_*.png')) if wrist_right_dir.exists() else []

    return {
        'metadata': metadata,
        'frames': frames,
        'head_imgs': head_imgs,
        'wrist_left_imgs': wrist_left_imgs,
        'wrist_right_imgs': wrist_right_imgs,
    }


def build_lerobot_dataset(input_dir, output_dir, task_description, fps=30):
    """Build a LeRobot v2.1 dataset from raw bimanual episodes."""
    input_dir = Path(input_dir)
    output_dir = Path(output_dir)

    # Find all episodes
    episode_dirs = sorted(input_dir.glob('episode_*'))
    if not episode_dirs:
        print(f"No episodes found in {input_dir}")
        sys.exit(1)

    print(f"Found {len(episode_dirs)} episodes")

    # Create output structure
    data_dir = output_dir / 'data' / 'chunk-000'
    meta_dir = output_dir / 'meta'
    data_dir.mkdir(parents=True, exist_ok=True)
    meta_dir.mkdir(parents=True, exist_ok=True)
    (meta_dir / 'episodes').mkdir(exist_ok=True)

    episodes_meta = []
    episodes_stats = []
    total_frames = 0
    task_id = 0

    for ep_idx, ep_dir in enumerate(episode_dirs):
        print(f"  Processing episode {ep_idx}: {ep_dir.name}...")
        raw = load_episode_raw(ep_dir)
        n_frames = len(raw['frames'])

        if n_frames < 10:
            print(f"    Skipping (too short: {n_frames} frames)")
            continue

        # Build parquet columns
        states = []
        actions = []
        head_images = []
        wrist_left_images = []
        wrist_right_images = []
        timestamps = []
        frame_indices = []
        episode_indices = []
        task_indices = []
        global_indices = []

        for i in range(n_frames):
            pos = raw['frames'][i]['positions']
            states.append(pos)

            # Action = next state (simple action representation)
            if i < n_frames - 1:
                actions.append(raw['frames'][i + 1]['positions'])
            else:
                actions.append(pos)  # Last frame: action = current position

            timestamps.append(i / fps)
            frame_indices.append(i)
            episode_indices.append(ep_idx)
            task_indices.append(task_id)
            global_indices.append(total_frames + i)

            # Load images as PNG bytes
            if i < len(raw['head_imgs']):
                with open(raw['head_imgs'][i], 'rb') as f:
                    head_images.append(f.read())
            else:
                head_images.append(b'')

            if i < len(raw['wrist_left_imgs']):
                with open(raw['wrist_left_imgs'][i], 'rb') as f:
                    wrist_left_images.append(f.read())
            else:
                wrist_left_images.append(b'')

            if i < len(raw['wrist_right_imgs']):
                with open(raw['wrist_right_imgs'][i], 'rb') as f:
                    wrist_right_images.append(f.read())
            else:
                wrist_right_images.append(b'')

        # Create parquet table
        states_arr = np.array(states, dtype=np.float32)
        actions_arr = np.array(actions, dtype=np.float32)

        table = pa.table({
            'observation.state': [s.tolist() for s in states_arr],
            'action': [a.tolist() for a in actions_arr],
            'observation.images.head': head_images,
            'observation.images.wrist_left': wrist_left_images,
            'observation.images.wrist_right': wrist_right_images,
            'episode_index': episode_indices,
            'frame_index': frame_indices,
            'timestamp': timestamps,
            'task_index': task_indices,
            'index': global_indices,
        })

        # Save per-episode parquet
        pq.write_table(table, data_dir / f'episode_{ep_idx:06d}.parquet')

        # Compute per-episode stats
        ep_stats = {
            'episode_index': ep_idx,
            'stats': {
                'observation.state': {
                    'mean': states_arr.mean(axis=0).tolist(),
                    'std': states_arr.std(axis=0).tolist(),
                    'min': states_arr.min(axis=0).tolist(),
                    'max': states_arr.max(axis=0).tolist(),
                    'count': [n_frames],
                },
                'action': {
                    'mean': actions_arr.mean(axis=0).tolist(),
                    'std': actions_arr.std(axis=0).tolist(),
                    'min': actions_arr.min(axis=0).tolist(),
                    'max': actions_arr.max(axis=0).tolist(),
                    'count': [n_frames],
                },
            }
        }
        episodes_stats.append(ep_stats)

        episodes_meta.append({
            'episode_index': ep_idx,
            'length': n_frames,
            'from': total_frames,
            'to': total_frames + n_frames,
        })

        total_frames += n_frames

    # Write metadata
    n_episodes = len(episodes_meta)

    # info.json
    info = {
        'codebase_version': '3.0',
        'robot_type': 'openarm_bimanual',
        'total_episodes': n_episodes,
        'total_frames': total_frames,
        'fps': fps,
        'data_path': 'data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet',
        'splits': {'train': f'0:{n_episodes}'},
        'features': {
            'observation.state': {
                'dtype': 'float32',
                'shape': [16],
                'names': [
                    'left_joint1', 'left_joint2', 'left_joint3', 'left_joint4',
                    'left_joint5', 'left_joint6', 'left_joint7', 'left_gripper',
                    'right_joint1', 'right_joint2', 'right_joint3', 'right_joint4',
                    'right_joint5', 'right_joint6', 'right_joint7', 'right_gripper',
                ],
            },
            'action': {
                'dtype': 'float32',
                'shape': [16],
                'names': [
                    'left_joint1', 'left_joint2', 'left_joint3', 'left_joint4',
                    'left_joint5', 'left_joint6', 'left_joint7', 'left_gripper',
                    'right_joint1', 'right_joint2', 'right_joint3', 'right_joint4',
                    'right_joint5', 'right_joint6', 'right_joint7', 'right_gripper',
                ],
            },
            'observation.images.head': {
                'dtype': 'image',
                'shape': [480, 640, 3],
                'names': ['height', 'width', 'channel'],
            },
            'observation.images.wrist_left': {
                'dtype': 'image',
                'shape': [480, 640, 3],
                'names': ['height', 'width', 'channel'],
            },
            'observation.images.wrist_right': {
                'dtype': 'image',
                'shape': [480, 640, 3],
                'names': ['height', 'width', 'channel'],
            },
        },
        'chunks_size': 1000,
        'task_to_task_index': {task_description: task_id},
    }

    with open(meta_dir / 'info.json', 'w') as f:
        json.dump(info, f, indent=4)

    # episodes.jsonl
    with open(meta_dir / 'episodes.jsonl', 'w') as f:
        for ep in episodes_meta:
            f.write(json.dumps(ep) + '\n')

    # episodes_stats.jsonl
    with open(meta_dir / 'episodes_stats.jsonl', 'w') as f:
        for st in episodes_stats:
            f.write(json.dumps(st) + '\n')

    # tasks.jsonl
    with open(meta_dir / 'tasks.jsonl', 'w') as f:
        f.write(json.dumps({'task_index': task_id, 'task': task_description}) + '\n')

    print(f"\n✅ Dataset created at: {output_dir}")
    print(f"   Episodes: {n_episodes}")
    print(f"   Total frames: {total_frames}")
    print(f"   State/Action dim: 16 (bimanual)")
    print(f"   Cameras: head, wrist_left, wrist_right")


def main():
    parser = argparse.ArgumentParser(
        description='Convert raw bimanual episodes to LeRobot format')
    parser.add_argument('--input', required=True,
                        help='Directory with raw episodes')
    parser.add_argument('--output', required=True,
                        help='Output LeRobot dataset directory')
    parser.add_argument('--task', required=True,
                        help='Task description')
    parser.add_argument('--fps', type=int, default=30,
                        help='Recording FPS (default: 30)')
    args = parser.parse_args()

    build_lerobot_dataset(args.input, args.output, args.task, args.fps)


if __name__ == '__main__':
    main()
