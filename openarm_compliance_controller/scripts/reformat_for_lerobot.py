#!/usr/bin/env python3
"""Reformat our dataset to match LeRobot v0.5.2.

Embeds actual image bytes into parquet using HF Dataset's Image feature.
Processes in chunks to avoid OOM.

Usage:
    python3 reformat_for_lerobot.py --root ~/lerobot_dataset
"""

import argparse
import json
import gc
from pathlib import Path

import datasets
import numpy as np
import pandas as pd
import pyarrow as pa
import pyarrow.parquet as pq
from PIL import Image


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', required=True)
    args = parser.parse_args()

    root = Path(args.root)
    meta = root / 'meta'

    print(f"Reformatting {root} for LeRobot v0.5.2...")

    # ── 1. Fix info.json ──────────────────────────────────
    info_path = meta / 'info.json'
    with open(info_path) as f:
        info = json.load(f)

    for key in ['action_type', 'repo_id']:
        info.pop(key, None)

    info['total_tasks'] = 1
    info['chunks_size'] = 1000
    info['data_path'] = 'data/chunk-{chunk_index:03d}/file-{file_index:03d}.parquet'
    info['splits'] = {'train': f'0:{info["total_episodes"]}'}

    for key, feat in info.get('features', {}).items():
        feat.setdefault('names', None)

    with open(info_path, 'w') as f:
        json.dump(info, f, indent=2)
    print("  ✅ info.json")

    # ── 2. Tasks parquet ──────────────────────────────────
    tasks_parquet = meta / 'tasks.parquet'
    if not tasks_parquet.exists():
        tasks_jsonl = meta / 'tasks.jsonl'
        if tasks_jsonl.exists():
            tasks = []
            with open(tasks_jsonl) as f:
                for line in f:
                    tasks.append(json.loads(line.strip()))
            df_t = pd.DataFrame(tasks)
            if 'task_index' in df_t.columns:
                df_t = df_t.set_index('task_index')
            df_t.index.name = 'task_index'
            df_t.to_parquet(tasks_parquet)
    print("  ✅ tasks.parquet")

    # ── 3. Episodes parquet ───────────────────────────────
    ep_dir = meta / 'episodes' / 'chunk-000'
    ep_dir.mkdir(parents=True, exist_ok=True)
    ep_path = ep_dir / 'file-000.parquet'

    episodes_jsonl = meta / 'episodes.jsonl'
    if episodes_jsonl.exists():
        episodes = []
        with open(episodes_jsonl) as f:
            for line in f:
                episodes.append(json.loads(line.strip()))
        df_ep_raw = pd.DataFrame(episodes)
    elif ep_path.exists():
        df_ep_raw = pd.read_parquet(ep_path)
    else:
        raise FileNotFoundError("No episodes source")

    ep_records = []
    for _, row in df_ep_raw.iterrows():
        from_idx = int(row.get('from', row.get('dataset_from_index', 0)))
        to_idx = int(row.get('to', row.get('dataset_to_index', from_idx + row['length'])))
        ep_records.append({
            'episode_index': int(row['episode_index']),
            'tasks': ['Pick up the bottle'],
            'length': int(row['length']),
            'dataset_from_index': from_idx,
            'dataset_to_index': to_idx,
            'data/chunk_index': 0,
            'data/file_index': 0,
            'meta/episodes/chunk_index': 0,
            'meta/episodes/file_index': 0,
        })
    df_episodes = pd.DataFrame(ep_records)
    df_episodes.to_parquet(ep_path, index=False)
    print(f"  ✅ episodes parquet ({len(df_episodes)})")

    # ── 4. Rebuild data parquet with embedded images ──────
    data_dir = root / 'data' / 'chunk-000'
    data_path = data_dir / 'file-000.parquet'
    if not data_path.exists():
        old = root / 'data' / 'train-00000-of-00001.parquet'
        if old.exists():
            data_dir.mkdir(parents=True, exist_ok=True)
            old.rename(data_path)

    # Read just the non-image columns first
    df = pd.read_parquet(data_path)
    total = len(df)
    print(f"  {total} frames to process")

    # Merge split columns
    state_cols = sorted([c for c in df.columns if c.startswith('observation.state.')])
    if state_cols:
        df['observation.state'] = df[state_cols].values.astype(np.float32).tolist()
        df = df.drop(columns=state_cols)

    action_cols = sorted([c for c in df.columns if c.startswith('action.')])
    if action_cols:
        df['action'] = df[action_cols].values.astype(np.float32).tolist()
        df = df.drop(columns=action_cols)

    if 'task' in df.columns and 'task_index' not in df.columns:
        df['task_index'] = 0
        df = df.drop(columns=['task'])

    if 'index' not in df.columns:
        df['index'] = range(len(df))

    # Extract image paths
    image_keys = ['observation.images.head_cam', 'observation.images.wrist_cam']
    head_paths = df['observation.images.head_cam'].tolist() if isinstance(df['observation.images.head_cam'].iloc[0], str) else None
    wrist_paths = df['observation.images.wrist_cam'].tolist() if isinstance(df['observation.images.wrist_cam'].iloc[0], str) else None

    # Drop image columns from df to save memory
    for k in image_keys:
        if k in df.columns:
            df = df.drop(columns=[k])

    # HF features
    hf_features = datasets.Features({
        'observation.state': datasets.Sequence(datasets.Value('float32'), length=8),
        'action': datasets.Sequence(datasets.Value('float32'), length=8),
        'observation.images.head_cam': datasets.Image(),
        'observation.images.wrist_cam': datasets.Image(),
        'episode_index': datasets.Value('int64'),
        'frame_index': datasets.Value('int64'),
        'timestamp': datasets.Value('float64'),
        'task_index': datasets.Value('int64'),
        'index': datasets.Value('int64'),
    })

    # Process in chunks of 2000 frames
    CHUNK = 2000
    output_path = data_dir / 'file-000-new.parquet'
    writer = None

    for start in range(0, total, CHUNK):
        end = min(start + CHUNK, total)
        chunk_df = df.iloc[start:end].copy()

        # Load images for this chunk
        head_imgs = []
        wrist_imgs = []
        for i in range(start, end):
            if head_paths:
                try:
                    head_imgs.append(Image.open(head_paths[i]).convert('RGB'))
                except:
                    head_imgs.append(Image.new('RGB', (640, 480)))
            if wrist_paths:
                try:
                    wrist_imgs.append(Image.open(wrist_paths[i]).convert('RGB'))
                except:
                    wrist_imgs.append(Image.new('RGB', (640, 480)))

        chunk_df['observation.images.head_cam'] = head_imgs
        chunk_df['observation.images.wrist_cam'] = wrist_imgs

        # Order columns
        cols = ['observation.state', 'action', 'observation.images.head_cam',
                'observation.images.wrist_cam', 'episode_index', 'frame_index',
                'timestamp', 'task_index', 'index']
        chunk_df = chunk_df[cols]

        # Convert to HF Dataset and get Arrow table
        ds = datasets.Dataset.from_dict(chunk_df.to_dict(orient='list'), features=hf_features)
        table = ds.data.table

        if writer is None:
            writer = pq.ParquetWriter(str(output_path), table.schema)
        writer.write_table(table)

        del chunk_df, head_imgs, wrist_imgs, ds, table
        gc.collect()

        print(f"    {end}/{total} frames written")

    if writer:
        writer.close()

    # Replace old file
    data_path.unlink()
    output_path.rename(data_path)

    print(f"  ✅ data parquet with embedded images ({total} frames)")
    print(f"\n✅ Reformat complete!")


if __name__ == '__main__':
    main()
