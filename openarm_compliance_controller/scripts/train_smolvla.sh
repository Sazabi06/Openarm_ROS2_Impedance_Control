#!/usr/bin/env bash
# train_smolvla.sh - SmolVLA baseline fine-tuning script for OpenArm
# Hardware Target: RTX 5080 (16GB VRAM)

echo "Starting SmolVLA fine-tuning..."

# 1. Setup Environment
export PATH="$HOME/miniforge3/bin:$PATH"
eval "$(conda shell.bash hook)"
conda activate lerobot

# 2. Cleanup previous runs (optional, comment out to keep history)
rm -rf ~/smolvla_finetuned

# 3. Weights & Biases Logging Note
echo "================================================================"
echo "To monitor training loss and metrics in your browser:"
echo "1. Run 'wandb login' and paste your API key (from wandb.ai/authorize)"
echo "2. The terminal will print a link to your live dashboard"
echo "If you prefer not to use WandB, the loss will still print to this"
echo "terminal every 50 steps."
echo "================================================================"

# 4. Training Command
#
# Hyperparameter Choices for 50 Episodes (RTX 5080 16GB):
# - batch_size=8: Fits comfortably in 16GB VRAM while maintaining good gradient estimates.
# - steps=10000: Since 50 episodes is a small dataset (~31k frames), 10k steps 
#   is ~2.5 epochs. This is long enough to learn but stops before heavy overfitting.
# - save_freq=2500: Saves 4 checkpoints over the run so we can pick the best 
#   one if the model starts to overfit near the end.
# - log_freq=50: Prints loss frequently so we can monitor the steep initial drop.
# - policy.empty_cameras=1: We have 2 cameras (head, wrist), but SmolVLA base 
#   expects 3. This pads the missing camera.
# - policy.use_amp=true: Enables Automatic Mixed Precision (bfloat16/float16)
#   to significantly reduce VRAM usage and speed up training on RTX 50 series.
# - dataset.image_transforms.enable=true: Enables data augmentation (ColorJitter, 
#   RandomAffine) to improve robustness given the small dataset size.

lerobot-train \
  --policy.path=lerobot/smolvla_base \
  --policy.push_to_hub=false \
  --dataset.repo_id=local/openarm_bottle_pickup \
  --dataset.root=$HOME/lerobot_dataset \
  --batch_size=8 \
  --steps=10000 \
  --save_freq=2500 \
  --log_freq=50 \
  --output_dir=$HOME/smolvla_finetuned \
  --policy.empty_cameras=1 \
  --policy.use_amp=true \
  --dataset.image_transforms.enable=true \
  --wandb.enable=true \
  --wandb.project=SmolVLA_Fine_Tunning \
  '--rename_map={"observation.images.head_cam": "observation.images.camera1", "observation.images.wrist_cam": "observation.images.camera2"}'

echo "Training complete! Checkpoints saved in ~/smolvla_finetuned"
