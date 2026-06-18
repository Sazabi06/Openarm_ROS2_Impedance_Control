#!/bin/bash
# LeRobot + SmolVLA Environment Setup for OpenArm
# Agent-C2 (Vision/VLA) — Phase 3, Task 3.1
# Date: 2026-05-06
#
# USAGE:
#   1. Run this script ONCE to set up the environment
#   2. For daily use: conda activate lerobot
#   3. CAN calibration requires ROS 2 stopped (one-time)
#
# VERIFIED:
#   - LeRobot v0.5.2
#   - SmolVLA policy imports OK
#   - PyTorch 2.10.0+cu128
#   - CUDA: NVIDIA GeForce RTX 5080 Laptop GPU (16.6 GB VRAM)

set -e

echo "═══════════════════════════════════════════════════"
echo "  LeRobot + SmolVLA Setup for OpenArm"
echo "═══════════════════════════════════════════════════"

# ── Step 1: Miniforge ───────────────────────────────────
if [ ! -d "$HOME/miniforge3" ]; then
    echo "[1/5] Installing Miniforge..."
    wget -q "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-x86_64.sh" -O /tmp/miniforge.sh
    bash /tmp/miniforge.sh -b -p ~/miniforge3
    rm /tmp/miniforge.sh
else
    echo "[1/5] Miniforge already installed ✓"
fi

export PATH="$HOME/miniforge3/bin:$PATH"
eval "$(conda shell.bash hook)"

# ── Step 2: Conda Environment ──────────────────────────
if conda env list | grep -q "lerobot"; then
    echo "[2/5] Conda 'lerobot' env already exists ✓"
else
    echo "[2/5] Creating conda 'lerobot' environment (Python 3.12)..."
    conda create -y -n lerobot python=3.12
fi

conda activate lerobot
conda install -y pip ffmpeg -c conda-forge

# ── Step 3: Clone LeRobot ──────────────────────────────
if [ ! -d "$HOME/lerobot" ]; then
    echo "[3/5] Cloning LeRobot..."
    cd ~
    git clone https://github.com/huggingface/lerobot.git
else
    echo "[3/5] LeRobot already cloned ✓"
fi

# ── Step 4: Install with SmolVLA + DaMiao ──────────────
echo "[4/5] Installing LeRobot with SmolVLA + DaMiao extras..."
cd ~/lerobot
pip install -e ".[smolvla,damiao]"

# ── Step 5: Verify ─────────────────────────────────────
echo "[5/5] Verifying installation..."
python -c "
from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
from lerobot.policies.smolvla.configuration_smolvla import SmolVLAConfig
from lerobot.__version__ import __version__
import torch
print(f'  LeRobot: v{__version__}')
print(f'  SmolVLA: OK')
print(f'  PyTorch: {torch.__version__}')
print(f'  CUDA: {torch.cuda.is_available()}')
if torch.cuda.is_available():
    print(f'  GPU: {torch.cuda.get_device_name(0)}')
    print(f'  VRAM: {torch.cuda.get_device_properties(0).total_memory / 1e9:.1f} GB')
print()
print('  ✅ All checks passed!')
"

echo ""
echo "═══════════════════════════════════════════════════"
echo "  Setup Complete!"
echo ""
echo "  To use:  conda activate lerobot"
echo ""
echo "  CAN Calibration (one-time, requires ROS 2 stopped):"
echo "    1. Stop ROS 2 launch (Ctrl+C Terminal 1)"
echo "    2. conda activate lerobot"
echo "    3. lerobot-setup-can --mode=setup --interfaces=can0,can1"
echo "    4. lerobot-calibrate \\"
echo "         --robot.type=openarm_follower \\"
echo "         --robot.port=can0 \\"
echo "         --robot.side=right \\"
echo "         --robot.id=openarm_right"
echo "    5. Restart ROS 2 launch"
echo "═══════════════════════════════════════════════════"
