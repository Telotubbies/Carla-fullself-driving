#!/usr/bin/env python3
"""
Visualize Training Progress
Plot training metrics from TensorBoard logs
"""

import argparse
import os
import sys
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

try:
    from tensorboard.backend.event_processing.event_accumulator import EventAccumulator
    TENSORBOARD_AVAILABLE = True
except ImportError:
    TENSORBOARD_AVAILABLE = False
    print("⚠️  TensorBoard not available. Install with: pip install tensorboard")


def load_tensorboard_data(log_dir: str, scalar_name: str):
    """Load scalar data from TensorBoard logs"""
    if not TENSORBOARD_AVAILABLE:
        return None, None
    
    event_acc = EventAccumulator(log_dir)
    event_acc.Reload()
    
    if scalar_name not in event_acc.Tags()['scalars']:
        return None, None
    
    scalar_events = event_acc.Scalars(scalar_name)
    steps = [s.step for s in scalar_events]
    values = [s.value for s in scalar_events]
    
    return np.array(steps), np.array(values)


def plot_training_metrics(log_dir: str, output_file: str = None):
    """Plot training metrics from TensorBoard logs"""
    if not TENSORBOARD_AVAILABLE:
        print("❌ TensorBoard not available")
        return
    
    print(f"Loading training logs from: {log_dir}")
    
    # Metrics to plot
    metrics = {
        'train/rollout/ep_rew_mean': 'Episode Reward',
        'train/rollout/ep_len_mean': 'Episode Length',
        'train/loss/value_loss': 'Value Loss',
        'train/loss/policy_loss': 'Policy Loss',
    }
    
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    axes = axes.flatten()
    
    for idx, (metric_key, metric_name) in enumerate(metrics.items()):
        steps, values = load_tensorboard_data(log_dir, metric_key)
        
        if steps is not None and len(steps) > 0:
            axes[idx].plot(steps, values, linewidth=2)
            axes[idx].set_title(metric_name, fontsize=12, fontweight='bold')
            axes[idx].set_xlabel('Steps')
            axes[idx].set_ylabel(metric_name)
            axes[idx].grid(True, alpha=0.3)
        else:
            axes[idx].text(0.5, 0.5, f'No data for\n{metric_name}', 
                          ha='center', va='center', transform=axes[idx].transAxes)
            axes[idx].set_title(metric_name)
    
    plt.tight_layout()
    
    if output_file:
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"✅ Plot saved to: {output_file}")
    else:
        plt.show()


def main():
    parser = argparse.ArgumentParser(description='Visualize Training Progress')
    parser.add_argument(
        '--logdir',
        type=str,
        default='logs/tensorboard',
        help='Path to TensorBoard log directory'
    )
    parser.add_argument(
        '--output',
        type=str,
        default=None,
        help='Output file path for plot (optional)'
    )
    
    args = parser.parse_args()
    
    # Resolve path
    base_dir = Path(__file__).parent.parent
    log_dir = args.logdir if os.path.isabs(args.logdir) else os.path.join(base_dir, args.logdir)
    
    if not os.path.exists(log_dir):
        print(f"❌ Log directory not found: {log_dir}")
        sys.exit(1)
    
    plot_training_metrics(log_dir, args.output)


if __name__ == '__main__':
    main()

