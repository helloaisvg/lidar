#!/usr/bin/env python3
"""
简单的轨迹评估工具
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
import sys

def read_trajectory(filename):
    """读取TUM格式的轨迹文件"""
    trajectory = []
    timestamps = []
    
    with open(filename, 'r') as f:
        for line in f:
            if line.startswith('#'):
                continue
            parts = line.strip().split()
            if len(parts) >= 8:
                timestamp = float(parts[0])
                x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                qx, qy, qz, qw = float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])
                
                trajectory.append([x, y, z])
                timestamps.append(timestamp)
    
    return np.array(trajectory), np.array(timestamps)

def calculate_trajectory_stats(trajectory, timestamps):
    """计算轨迹统计信息"""
    if len(trajectory) == 0:
        return {}
    
    # 计算总距离
    distances = np.linalg.norm(np.diff(trajectory, axis=0), axis=1)
    total_distance = np.sum(distances)
    
    # 计算速度
    time_diffs = np.diff(timestamps)
    speeds = distances / time_diffs
    avg_speed = np.mean(speeds)
    
    # 计算范围
    x_range = np.max(trajectory[:, 0]) - np.min(trajectory[:, 0])
    y_range = np.max(trajectory[:, 1]) - np.min(trajectory[:, 1])
    z_range = np.max(trajectory[:, 2]) - np.min(trajectory[:, 2])
    
    return {
        'total_distance': total_distance,
        'avg_speed': avg_speed,
        'x_range': x_range,
        'y_range': y_range,
        'z_range': z_range,
        'num_points': len(trajectory),
        'duration': timestamps[-1] - timestamps[0] if len(timestamps) > 1 else 0
    }

def plot_trajectory_2d(trajectory, title, save_path):
    """绘制2D轨迹图"""
    plt.figure(figsize=(12, 8))
    plt.plot(trajectory[:, 0], trajectory[:, 1], 'b-', linewidth=2, label='Trajectory')
    plt.scatter(trajectory[0, 0], trajectory[0, 1], color='green', s=100, label='Start', zorder=5)
    plt.scatter(trajectory[-1, 0], trajectory[-1, 1], color='red', s=100, label='End', zorder=5)
    
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title(title)
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"2D轨迹图已保存: {save_path}")

def plot_trajectory_3d(trajectory, title, save_path):
    """绘制3D轨迹图"""
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    ax.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], 'b-', linewidth=2, label='Trajectory')
    ax.scatter(trajectory[0, 0], trajectory[0, 1], trajectory[0, 2], color='green', s=100, label='Start')
    ax.scatter(trajectory[-1, 0], trajectory[-1, 1], trajectory[-1, 2], color='red', s=100, label='End')
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title(title)
    ax.legend()
    
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"3D轨迹图已保存: {save_path}")

def plot_speed_profile(trajectory, timestamps, title, save_path):
    """绘制速度曲线"""
    distances = np.linalg.norm(np.diff(trajectory, axis=0), axis=1)
    time_diffs = np.diff(timestamps)
    speeds = distances / time_diffs
    
    plt.figure(figsize=(12, 6))
    plt.plot(timestamps[1:], speeds, 'r-', linewidth=2)
    plt.xlabel('Time (s)')
    plt.ylabel('Speed (m/s)')
    plt.title(title)
    plt.grid(True, alpha=0.3)
    
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"速度曲线已保存: {save_path}")

def evaluate_trajectory(trajectory_file, output_dir):
    """评估轨迹文件"""
    print(f"评估轨迹文件: {trajectory_file}")
    
    # 读取轨迹
    trajectory, timestamps = read_trajectory(trajectory_file)
    
    if len(trajectory) == 0:
        print("错误: 轨迹文件为空")
        return
    
    # 计算统计信息
    stats = calculate_trajectory_stats(trajectory, timestamps)
    
    print(f"\n轨迹统计信息:")
    print(f"  轨迹点数: {stats['num_points']}")
    print(f"  总距离: {stats['total_distance']:.2f} m")
    print(f"  平均速度: {stats['avg_speed']:.2f} m/s")
    print(f"  持续时间: {stats['duration']:.2f} s")
    print(f"  X范围: {stats['x_range']:.2f} m")
    print(f"  Y范围: {stats['y_range']:.2f} m")
    print(f"  Z范围: {stats['z_range']:.2f} m")
    
    # 创建输出目录
    os.makedirs(output_dir, exist_ok=True)
    
    # 生成文件名
    base_name = os.path.splitext(os.path.basename(trajectory_file))[0]
    
    # 绘制图表
    plot_trajectory_2d(trajectory, f"2D Trajectory - {base_name}", 
                      os.path.join(output_dir, f"{base_name}_2d.png"))
    
    plot_trajectory_3d(trajectory, f"3D Trajectory - {base_name}", 
                      os.path.join(output_dir, f"{base_name}_3d.png"))
    
    plot_speed_profile(trajectory, timestamps, f"Speed Profile - {base_name}", 
                      os.path.join(output_dir, f"{base_name}_speed.png"))
    
    # 保存统计信息
    stats_file = os.path.join(output_dir, f"{base_name}_stats.txt")
    with open(stats_file, 'w') as f:
        f.write(f"Trajectory Statistics: {base_name}\n")
        f.write("=" * 50 + "\n")
        f.write(f"Number of points: {stats['num_points']}\n")
        f.write(f"Total distance: {stats['total_distance']:.2f} m\n")
        f.write(f"Average speed: {stats['avg_speed']:.2f} m/s\n")
        f.write(f"Duration: {stats['duration']:.2f} s\n")
        f.write(f"X range: {stats['x_range']:.2f} m\n")
        f.write(f"Y range: {stats['y_range']:.2f} m\n")
        f.write(f"Z range: {stats['z_range']:.2f} m\n")
    
    print(f"统计信息已保存: {stats_file}")
    print(f"所有结果保存在: {output_dir}")

def main():
    if len(sys.argv) < 2:
        print("用法: python3 simple_trajectory_evaluator.py <trajectory_file> [output_dir]")
        sys.exit(1)
    
    trajectory_file = sys.argv[1]
    output_dir = sys.argv[2] if len(sys.argv) > 2 else "trajectory_evaluation_results"
    
    if not os.path.exists(trajectory_file):
        print(f"错误: 轨迹文件不存在 {trajectory_file}")
        sys.exit(1)
    
    evaluate_trajectory(trajectory_file, output_dir)

if __name__ == "__main__":
    main()