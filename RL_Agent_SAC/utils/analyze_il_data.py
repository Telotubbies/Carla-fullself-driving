import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import argparse
import os
from pathlib import Path
plt.style.use('seaborn-v0_8-darkgrid')
sns.set_palette("husl")
def load_demo_data(demo_path: str):
    
    if not os.path.exists(demo_path):
        raise FileNotFoundError(f"Demo file not found: {demo_path}")
    data = np.load(demo_path)
    print(f"✅ Loaded: {demo_path}")
    print(f"   Keys: {list(data.keys())}")
    return data
def analyze_basic_stats(data):
    
    print("\n" + "="*70)
    print("📊 Basic Statistics")
    print("="*70)
    dones = data['dones']
    num_episodes = int(np.sum(dones)) + (1 if not dones[-1] else 0)
    total_steps = len(dones)
    print(f"\n📈 Dataset Overview:")
    print(f"   Total steps: {total_steps:,}")
    print(f"   Number of episodes: {num_episodes}")
    print(f"   Average steps per episode: {total_steps/num_episodes:.1f}")
    if 'actions' in data:
        actions = data['actions']
        print(f"\n🎮 Actions [steer, throttle, brake]:")
        print(f"   Shape: {actions.shape}")
        print(f"   Steer range: [{actions[:, 0].min():.3f}, {actions[:, 0].max():.3f}]")
        print(f"   Throttle range: [{actions[:, 1].min():.3f}, {actions[:, 1].max():.3f}]")
        print(f"   Brake range: [{actions[:, 2].min():.3f}, {actions[:, 2].max():.3f}]")
        print(f"   Steer mean: {actions[:, 0].mean():.3f} ± {actions[:, 0].std():.3f}")
        print(f"   Throttle mean: {actions[:, 1].mean():.3f} ± {actions[:, 1].std():.3f}")
        print(f"   Brake mean: {actions[:, 2].mean():.3f} ± {actions[:, 2].std():.3f}")
    if 'steers' in data:
        print(f"\n🎮 Separate Action Components:")
        print(f"   Steers: mean={data['steers'].mean():.3f}, std={data['steers'].std():.3f}")
        print(f"   Throttles: mean={data['throttles'].mean():.3f}, std={data['throttles'].std():.3f}")
        print(f"   Brakes: mean={data['brakes'].mean():.3f}, std={data['brakes'].std():.3f}")
    if 'speeds' in data:
        speeds = data['speeds']
        print(f"\n🚗 Speeds (km/h):")
        print(f"   Range: [{speeds.min():.2f}, {speeds.max():.2f}]")
        print(f"   Mean: {speeds.mean():.2f} ± {speeds.std():.2f}")
        print(f"   Median: {np.median(speeds):.2f}")
    if 'collisions' in data:
        collisions = data['collisions']
        num_collisions = int(np.sum(collisions))
        collision_rate = num_collisions / total_steps * 100
        print(f"\n💥 Collisions:")
        print(f"   Total collisions: {num_collisions}")
        print(f"   Collision rate: {collision_rate:.2f}%")
    if 'locations' in data:
        locations = data['locations']
        print(f"\n📍 Locations (x, y, z):")
        print(f"   X range: [{locations[:, 0].min():.2f}, {locations[:, 0].max():.2f}]")
        print(f"   Y range: [{locations[:, 1].min():.2f}, {locations[:, 1].max():.2f}]")
        print(f"   Z range: [{locations[:, 2].min():.2f}, {locations[:, 2].max():.2f}]")
    if 'velocities' in data:
        velocities = data['velocities']
        speeds_3d = np.linalg.norm(velocities, axis=1) * 3.6
        print(f"\n⚡ Velocities (3D, km/h):")
        print(f"   Speed range: [{speeds_3d.min():.2f}, {speeds_3d.max():.2f}]")
        print(f"   Mean speed: {speeds_3d.mean():.2f} ± {speeds_3d.std():.2f}")
def plot_actions(data, output_dir='plots'):
    
    os.makedirs(output_dir, exist_ok=True)
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle('Action Distributions', fontsize=16, fontweight='bold')
    if 'actions' in data:
        actions = data['actions']
        axes[0, 0].hist(actions[:, 0], bins=50, alpha=0.7, color='blue', edgecolor='black')
        axes[0, 0].set_title('Steer Distribution', fontweight='bold')
        axes[0, 0].set_xlabel('Steer Value')
        axes[0, 0].set_ylabel('Frequency')
        axes[0, 0].axvline(actions[:, 0].mean(), color='red', linestyle='--', label=f'Mean: {actions[:, 0].mean():.3f}')
        axes[0, 0].legend()
        axes[0, 0].grid(True, alpha=0.3)
        axes[0, 1].hist(actions[:, 1], bins=50, alpha=0.7, color='green', edgecolor='black')
        axes[0, 1].set_title('Throttle Distribution', fontweight='bold')
        axes[0, 1].set_xlabel('Throttle Value')
        axes[0, 1].set_ylabel('Frequency')
        axes[0, 1].axvline(actions[:, 1].mean(), color='red', linestyle='--', label=f'Mean: {actions[:, 1].mean():.3f}')
        axes[0, 1].legend()
        axes[0, 1].grid(True, alpha=0.3)
        axes[1, 0].hist(actions[:, 2], bins=50, alpha=0.7, color='red', edgecolor='black')
        axes[1, 0].set_title('Brake Distribution', fontweight='bold')
        axes[1, 0].set_xlabel('Brake Value')
        axes[1, 0].set_ylabel('Frequency')
        axes[1, 0].axvline(actions[:, 2].mean(), color='blue', linestyle='--', label=f'Mean: {actions[:, 2].mean():.3f}')
        axes[1, 0].legend()
        axes[1, 0].grid(True, alpha=0.3)
        action_df = {
            'Steer': actions[:, 0],
            'Throttle': actions[:, 1],
            'Brake': actions[:, 2]
        }
        corr = np.corrcoef([actions[:, 0], actions[:, 1], actions[:, 2]])
        im = axes[1, 1].imshow(corr, cmap='coolwarm', aspect='auto', vmin=-1, vmax=1)
        axes[1, 1].set_xticks([0, 1, 2])
        axes[1, 1].set_yticks([0, 1, 2])
        axes[1, 1].set_xticklabels(['Steer', 'Throttle', 'Brake'])
        axes[1, 1].set_yticklabels(['Steer', 'Throttle', 'Brake'])
        axes[1, 1].set_title('Action Correlation Matrix', fontweight='bold')
        plt.colorbar(im, ax=axes[1, 1])
        for i in range(3):
            for j in range(3):
                axes[1, 1].text(j, i, f'{corr[i, j]:.2f}',
                               ha='center', va='center', color='white' if abs(corr[i, j]) > 0.5 else 'black')
    plt.tight_layout()
    plt.savefig(f'{output_dir}/actions_distribution.png', dpi=150, bbox_inches='tight')
    print(f"\n✅ Saved: {output_dir}/actions_distribution.png")
    plt.close()
def plot_speeds(data, output_dir='plots'):
    
    os.makedirs(output_dir, exist_ok=True)
    if 'speeds' not in data:
        print("⚠️  No speed data available")
        return
    speeds = data['speeds']
    dones = data['dones']
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle('Speed Analysis', fontsize=16, fontweight='bold')
    axes[0, 0].hist(speeds, bins=50, alpha=0.7, color='purple', edgecolor='black')
    axes[0, 0].set_title('Speed Distribution', fontweight='bold')
    axes[0, 0].set_xlabel('Speed (km/h)')
    axes[0, 0].set_ylabel('Frequency')
    axes[0, 0].axvline(speeds.mean(), color='red', linestyle='--', label=f'Mean: {speeds.mean():.2f} km/h')
    axes[0, 0].axvline(np.median(speeds), color='green', linestyle='--', label=f'Median: {np.median(speeds):.2f} km/h')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)
    sample_size = min(1000, len(speeds))
    axes[0, 1].plot(speeds[:sample_size], alpha=0.7, linewidth=0.5)
    axes[0, 1].set_title(f'Speed Over Time (first {sample_size} steps)', fontweight='bold')
    axes[0, 1].set_xlabel('Step')
    axes[0, 1].set_ylabel('Speed (km/h)')
    axes[0, 1].grid(True, alpha=0.3)
    episode_speeds = []
    episode_idx = 0
    current_episode = []
    for i, done in enumerate(dones):
        current_episode.append(speeds[i])
        if done or i == len(dones) - 1:
            if len(current_episode) > 0:
                episode_speeds.append(np.mean(current_episode))
            current_episode = []
            episode_idx += 1
    axes[1, 0].bar(range(len(episode_speeds)), episode_speeds, alpha=0.7, color='orange')
    axes[1, 0].set_title('Average Speed per Episode', fontweight='bold')
    axes[1, 0].set_xlabel('Episode')
    axes[1, 0].set_ylabel('Average Speed (km/h)')
    axes[1, 0].grid(True, alpha=0.3, axis='y')
    if 'actions' in data:
        actions = data['actions']
        sample_size = min(5000, len(speeds))
        scatter = axes[1, 1].scatter(actions[:sample_size, 1], speeds[:sample_size],
                                     alpha=0.3, s=1, c=actions[:sample_size, 0], cmap='coolwarm')
        axes[1, 1].set_title('Speed vs Throttle (colored by steer)', fontweight='bold')
        axes[1, 1].set_xlabel('Throttle')
        axes[1, 1].set_ylabel('Speed (km/h)')
        axes[1, 1].grid(True, alpha=0.3)
        plt.colorbar(scatter, ax=axes[1, 1], label='Steer')
    plt.tight_layout()
    plt.savefig(f'{output_dir}/speed_analysis.png', dpi=150, bbox_inches='tight')
    print(f"✅ Saved: {output_dir}/speed_analysis.png")
    plt.close()
def plot_vehicle_state(data, output_dir='plots'):
    
    os.makedirs(output_dir, exist_ok=True)
    if 'locations' not in data:
        print("⚠️  No vehicle state data available")
        return
    locations = data['locations']
    dones = data['dones']
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle('Vehicle State Analysis', fontsize=16, fontweight='bold')
    sample_size = min(5000, len(locations))
    axes[0, 0].plot(locations[:sample_size, 0], locations[:sample_size, 1],
                   alpha=0.6, linewidth=0.5, color='blue')
    axes[0, 0].scatter(locations[0, 0], locations[0, 1], color='green', s=100, marker='o', label='Start', zorder=5)
    axes[0, 0].scatter(locations[sample_size-1, 0], locations[sample_size-1, 1],
                       color='red', s=100, marker='x', label='End', zorder=5)
    axes[0, 0].set_title(f'Vehicle Trajectory (first {sample_size} steps)', fontweight='bold')
    axes[0, 0].set_xlabel('X (meters)')
    axes[0, 0].set_ylabel('Y (meters)')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].set_aspect('equal')
    sample_size = min(1000, len(locations))
    axes[0, 1].plot(locations[:sample_size, 2], alpha=0.7, linewidth=0.5, color='purple')
    axes[0, 1].set_title(f'Height (Z) Over Time (first {sample_size} steps)', fontweight='bold')
    axes[0, 1].set_xlabel('Step')
    axes[0, 1].set_ylabel('Height (meters)')
    axes[0, 1].grid(True, alpha=0.3)
    if 'rotations' in data:
        rotations = data['rotations']
        sample_size = min(1000, len(rotations))
        axes[1, 0].plot(rotations[:sample_size, 1], alpha=0.7, linewidth=0.5, color='orange')
        axes[1, 0].set_title(f'Yaw Angle Over Time (first {sample_size} steps)', fontweight='bold')
        axes[1, 0].set_xlabel('Step')
        axes[1, 0].set_ylabel('Yaw (degrees)')
        axes[1, 0].grid(True, alpha=0.3)
    if 'velocities' in data:
        velocities = data['velocities']
        speed_3d = np.linalg.norm(velocities, axis=1) * 3.6
        sample_size = min(1000, len(speed_3d))
        axes[1, 1].plot(speed_3d[:sample_size], alpha=0.7, linewidth=0.5, color='red')
        axes[1, 1].set_title(f'3D Speed Over Time (first {sample_size} steps)', fontweight='bold')
        axes[1, 1].set_xlabel('Step')
        axes[1, 1].set_ylabel('Speed (km/h)')
        axes[1, 1].grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(f'{output_dir}/vehicle_state.png', dpi=150, bbox_inches='tight')
    print(f"✅ Saved: {output_dir}/vehicle_state.png")
    plt.close()
def check_data_quality(data):
    
    print("\n" + "="*70)
    print("🔍 Data Quality Check")
    print("="*70)
    issues = []
    for key in data.keys():
        arr = data[key]
        if isinstance(arr, np.ndarray):
            nan_count = np.isnan(arr).sum()
            if nan_count > 0:
                issues.append(f"⚠️  {key}: {nan_count} NaN values")
    for key in data.keys():
        arr = data[key]
        if isinstance(arr, np.ndarray):
            inf_count = np.isinf(arr).sum()
            if inf_count > 0:
                issues.append(f"⚠️  {key}: {inf_count} infinite values")
    if 'actions' in data:
        actions = data['actions']
        if actions[:, 0].min() < -1.0 or actions[:, 0].max() > 1.0:
            issues.append("⚠️  Steer values out of range [-1, 1]")
        if actions[:, 1].min() < 0.0 or actions[:, 1].max() > 1.0:
            issues.append("⚠️  Throttle values out of range [0, 1]")
        if actions[:, 2].min() < 0.0 or actions[:, 2].max() > 1.0:
            issues.append("⚠️  Brake values out of range [0, 1]")
    if issues:
        print("\n⚠️  Issues found:")
        for issue in issues:
            print(f"   {issue}")
    else:
        print("\n✅ No data quality issues found!")
    return len(issues) == 0
def main():
    parser = argparse.ArgumentParser(description='Analyze IL demonstration data')
    parser.add_argument('--demo-file', type=str, required=True, help='Path to .npz demo file')
    parser.add_argument('--output-dir', type=str, default='plots', help='Output directory for plots')
    parser.add_argument('--no-plots', action='store_true', help='Skip plotting')
    args = parser.parse_args()
    data = load_demo_data(args.demo_file)
    analyze_basic_stats(data)
    is_clean = check_data_quality(data)
    if not args.no_plots:
        print("\n" + "="*70)
        print("📊 Generating Plots")
        print("="*70)
        plot_actions(data, args.output_dir)
        plot_speeds(data, args.output_dir)
        plot_vehicle_state(data, args.output_dir)
    print("\n" + "="*70)
    print("✅ Analysis Complete!")
    print("="*70)
    print(f"\n📁 Plots saved to: {args.output_dir}/")
    print(f"   • actions_distribution.png")
    print(f"   • speed_analysis.png")
    print(f"   • vehicle_state.png")
if __name__ == '__main__':
    main()