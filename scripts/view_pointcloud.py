#!/usr/bin/env python3
"""
点云可视化工具
使用matplotlib和mayavi来显示PCD点云文件
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys
import os

def read_pcd_file(filename):
    """读取PCD文件"""
    points = []
    
    with open(filename, 'r') as f:
        # 跳过头部信息
        for line in f:
            if line.startswith('POINTS'):
                num_points = int(line.split()[1])
                break
        
        # 读取点云数据
        for line in f:
            if line.strip():
                parts = line.strip().split()
                if len(parts) >= 3:
                    x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                    points.append([x, y, z])
    
    return np.array(points)

def plot_pointcloud_matplotlib(points, title, save_path=None):
    """使用matplotlib绘制点云"""
    fig = plt.figure(figsize=(15, 10))
    
    # 3D散点图
    ax1 = fig.add_subplot(221, projection='3d')
    ax1.scatter(points[:, 0], points[:, 1], points[:, 2], c=points[:, 2], 
               cmap='viridis', s=1, alpha=0.6)
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.set_title(f'{title} - 3D View')
    
    # XY平面投影
    ax2 = fig.add_subplot(222)
    scatter = ax2.scatter(points[:, 0], points[:, 1], c=points[:, 2], 
                         cmap='viridis', s=1, alpha=0.6)
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.set_title(f'{title} - XY Projection')
    ax2.set_aspect('equal')
    plt.colorbar(scatter, ax=ax2, label='Z (m)')
    
    # XZ平面投影
    ax3 = fig.add_subplot(223)
    scatter = ax3.scatter(points[:, 0], points[:, 2], c=points[:, 1], 
                         cmap='plasma', s=1, alpha=0.6)
    ax3.set_xlabel('X (m)')
    ax3.set_ylabel('Z (m)')
    ax3.set_title(f'{title} - XZ Projection')
    plt.colorbar(scatter, ax=ax3, label='Y (m)')
    
    # YZ平面投影
    ax4 = fig.add_subplot(224)
    scatter = ax4.scatter(points[:, 1], points[:, 2], c=points[:, 0], 
                         cmap='cool', s=1, alpha=0.6)
    ax4.set_xlabel('Y (m)')
    ax4.set_ylabel('Z (m)')
    ax4.set_title(f'{title} - YZ Projection')
    plt.colorbar(scatter, ax=ax4, label='X (m)')
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"点云可视化图已保存: {save_path}")
    
    plt.show()

def analyze_pointcloud(points):
    """分析点云统计信息"""
    print(f"点云统计信息:")
    print(f"  总点数: {len(points)}")
    print(f"  X范围: [{np.min(points[:, 0]):.2f}, {np.max(points[:, 0]):.2f}] m")
    print(f"  Y范围: [{np.min(points[:, 1]):.2f}, {np.max(points[:, 1]):.2f}] m")
    print(f"  Z范围: [{np.min(points[:, 2]):.2f}, {np.max(points[:, 2]):.2f}] m")
    print(f"  X跨度: {np.max(points[:, 0]) - np.min(points[:, 0]):.2f} m")
    print(f"  Y跨度: {np.max(points[:, 1]) - np.min(points[:, 1]):.2f} m")
    print(f"  Z跨度: {np.max(points[:, 2]) - np.min(points[:, 2]):.2f} m")
    print(f"  中心点: ({np.mean(points[:, 0]):.2f}, {np.mean(points[:, 1]):.2f}, {np.mean(points[:, 2]):.2f})")

def main():
    if len(sys.argv) < 2:
        print("用法: python3 view_pointcloud.py <pcd_file> [save_image]")
        print("示例: python3 view_pointcloud.py ndt_global_map_0.pcd")
        print("示例: python3 view_pointcloud.py ndt_global_map_0.pcd output.png")
        sys.exit(1)
    
    pcd_file = sys.argv[1]
    save_image = sys.argv[2] if len(sys.argv) > 2 else None
    
    if not os.path.exists(pcd_file):
        print(f"错误: PCD文件不存在 {pcd_file}")
        sys.exit(1)
    
    print(f"读取PCD文件: {pcd_file}")
    points = read_pcd_file(pcd_file)
    
    if len(points) == 0:
        print("错误: PCD文件为空")
        sys.exit(1)
    
    # 分析点云
    analyze_pointcloud(points)
    
    # 可视化点云
    title = os.path.splitext(os.path.basename(pcd_file))[0]
    plot_pointcloud_matplotlib(points, title, save_image)
    
    print(f"点云可视化完成!")

if __name__ == "__main__":
    main()

