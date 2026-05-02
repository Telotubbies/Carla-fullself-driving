import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from pathlib import Path
from typing import List, Optional
import cv2


def plot_training_curves(
    metrics_file: str,
    output_dir: str = "data/plots",
    show: bool = False
):
    """
    Plot training curves from metrics CSV file.
    
    Args:
        metrics_file: Path to training metrics CSV
        output_dir: Directory to save plots
        show: Whether to display plots
    """
    
    # Load metrics
    df = pd.read_csv(metrics_file)
    
    # Create output directory
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    
    # Plot reward curves
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    
    # Reward mean
    axes[0, 0].plot(df['iteration'], df['episode_reward_mean'], label='Mean Reward')
    axes[0, 0].fill_between(
        df['iteration'],
        df['episode_reward_min'],
        df['episode_reward_max'],
        alpha=0.3,
        label='Min-Max Range'
    )
    axes[0, 0].set_xlabel('Iteration')
    axes[0, 0].set_ylabel('Reward')
    axes[0, 0].set_title('Episode Reward')
    axes[0, 0].legend()
    axes[0, 0].grid(True)
    
    # Episode length
    axes[0, 1].plot(df['iteration'], df['episode_len_mean'], color='orange')
    axes[0, 1].set_xlabel('Iteration')
    axes[0, 1].set_ylabel('Steps')
    axes[0, 1].set_title('Episode Length')
    axes[0, 1].grid(True)
    
    # Collision rate
    if 'collision_rate' in df.columns:
        axes[1, 0].plot(df['iteration'], df['collision_rate'], color='red')
        axes[1, 0].set_xlabel('Iteration')
        axes[1, 0].set_ylabel('Rate')
        axes[1, 0].set_title('Collision Rate')
        axes[1, 0].grid(True)
    
    # Success rate
    if 'success_rate' in df.columns:
        axes[1, 1].plot(df['iteration'], df['success_rate'], color='green')
        axes[1, 1].set_xlabel('Iteration')
        axes[1, 1].set_ylabel('Rate')
        axes[1, 1].set_title('Success Rate')
        axes[1, 1].grid(True)
    
    plt.tight_layout()
    
    # Save plot
    plot_file = output_path / 'training_curves.png'
    plt.savefig(plot_file, dpi=300, bbox_inches='tight')
    print(f"Training curves saved to: {plot_file}")
    
    if show:
        plt.show()
    else:
        plt.close()


def plot_reward_distribution(
    rewards: List[float],
    output_file: str = "data/plots/reward_distribution.png",
    show: bool = False
):
    """
    Plot reward distribution histogram.
    
    Args:
        rewards: List of episode rewards
        output_file: Output file path
        show: Whether to display plot
    """
    
    plt.figure(figsize=(10, 6))
    plt.hist(rewards, bins=30, edgecolor='black', alpha=0.7)
    plt.xlabel('Episode Reward')
    plt.ylabel('Frequency')
    plt.title('Reward Distribution')
    plt.axvline(np.mean(rewards), color='red', linestyle='--', label=f'Mean: {np.mean(rewards):.2f}')
    plt.axvline(np.median(rewards), color='green', linestyle='--', label=f'Median: {np.median(rewards):.2f}')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    # Save plot
    output_path = Path(output_file)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"Reward distribution saved to: {output_file}")
    
    if show:
        plt.show()
    else:
        plt.close()


def save_episode_video(
    frames: List[np.ndarray],
    output_file: str,
    fps: int = 20
):
    """
    Save episode frames as video.
    
    Args:
        frames: List of image frames (numpy arrays)
        output_file: Output video file path
        fps: Frames per second
    """
    
    if not frames:
        print("No frames to save")
        return
    
    # Get frame dimensions
    height, width = frames[0].shape[:2]
    
    # Create video writer
    output_path = Path(output_file)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(str(output_file), fourcc, fps, (width, height))
    
    # Write frames
    for frame in frames:
        # Convert RGB to BGR for OpenCV
        if frame.shape[2] == 3:
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        else:
            frame_bgr = frame
        
        out.write(frame_bgr)
    
    out.release()
    print(f"Episode video saved to: {output_file}")


def plot_comparison(
    metrics_files: List[str],
    labels: List[str],
    output_file: str = "data/plots/comparison.png",
    show: bool = False
):
    """
    Plot comparison of multiple training runs.
    
    Args:
        metrics_files: List of paths to metrics CSV files
        labels: Labels for each run
        output_file: Output file path
        show: Whether to display plot
    """
    
    plt.figure(figsize=(12, 6))
    
    for metrics_file, label in zip(metrics_files, labels):
        df = pd.read_csv(metrics_file)
        plt.plot(df['iteration'], df['episode_reward_mean'], label=label)
    
    plt.xlabel('Iteration')
    plt.ylabel('Mean Episode Reward')
    plt.title('Training Comparison')
    plt.legend()
    plt.grid(True)
    
    # Save plot
    output_path = Path(output_file)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"Comparison plot saved to: {output_file}")
    
    if show:
        plt.show()
    else:
        plt.close()
