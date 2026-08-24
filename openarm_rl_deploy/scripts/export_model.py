#!/usr/bin/env python3
"""
将训练好的 Actor checkpoint 转换为 TorchScript 格式，供 C++ LibTorch 加载。

用法:
  python export_model.py --checkpoint models/jgzh_sim2real.pth --output models/jgzh_sim2real.pt
"""

import argparse
from pathlib import Path

import torch
import numpy as np


class Actor(torch.nn.Module):
    """与训练时完全一致的 Actor 网络结构。"""
    def __init__(self, state_dim=29, action_dim=4, hidden_dim=256):
        super().__init__()
        self.net = torch.nn.Sequential(
            torch.nn.Linear(state_dim, hidden_dim),
            torch.nn.ReLU(),
            torch.nn.Linear(hidden_dim, hidden_dim),
            torch.nn.ReLU(),
            torch.nn.Linear(hidden_dim, action_dim),
            torch.nn.Tanh(),
        )

    def forward(self, state):
        return self.net(state)


def export(checkpoint: str, output: str, state_dim: int, action_dim: int, hidden_dim: int):
    ckpt_path = Path(checkpoint)
    if not ckpt_path.exists():
        raise FileNotFoundError(f"Checkpoint not found: {ckpt_path}")

    out_path = Path(output)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    # 1. 创建与训练时相同结构的网络并加载权重
    device = torch.device("cpu")
    model = Actor(state_dim, action_dim, hidden_dim).to(device)
    state_dict = torch.load(str(ckpt_path), map_location=device, weights_only=True)
    model.load_state_dict(state_dict)
    model.eval()

    # 2. 验证：跑一次推理
    dummy_input = torch.randn(1, state_dim)
    with torch.no_grad():
        traced_output = model(dummy_input)
    print(f"[export] Test inference output shape: {traced_output.shape}")
    print(f"[export] Output range: [{traced_output.min().item():.4f}, {traced_output.max().item():.4f}]")

    # 3. TorchScript 追踪
    traced = torch.jit.trace(model, dummy_input)
    traced.save(str(out_path))

    # 4. 验证追踪后的模型
    loaded = torch.jit.load(str(out_path))
    with torch.no_grad():
        reloaded_output = loaded(dummy_input)
    diff = (traced_output - reloaded_output).abs().max().item()
    print(f"[export] Max diff after save/load: {diff:.2e}")

    size_kb = out_path.stat().st_size / 1024
    print(f"[export] Saved TorchScript model to: {out_path} ({size_kb:.0f} KB)")
    print(f"[export] Success! Use this .pt file with the C++ ROS2 node.")


def main():
    parser = argparse.ArgumentParser(description="Export Actor checkpoint to TorchScript")
    parser.add_argument("--checkpoint", default="models/jgzh_sim2real.pth")
    parser.add_argument("--output", default=None,
                        help="Output .pt path (default: checkpoint path with .pt extension)")
    parser.add_argument("--state-dim", type=int, default=29)
    parser.add_argument("--action-dim", type=int, default=4)
    parser.add_argument("--hidden-dim", type=int, default=256)
    args = parser.parse_args()

    output = args.output
    if output is None:
        output = str(Path(args.checkpoint).with_suffix(".pt"))

    export(args.checkpoint, output, args.state_dim, args.action_dim, args.hidden_dim)


if __name__ == "__main__":
    main()
