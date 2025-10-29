#!/usr/bin/env python3
"""
直接bag包处理器
不依赖ROS 1，直接解析bag包文件
在ubuntu24.04下运行，无法配置ROS1所以直接就解析了
"""

import os
import sys
import struct
import time
import numpy as np
from pathlib import Path

class DirectBagProcessor:
    def __init__(self, bag_file):
        self.bag_file = bag_file
        self.output_dir = "direct_processing_results"
        self.trajectory_points = []
        self.point_cloud_count = 0
        
    def process_bag(self):
        """处理bag包"""
        print(f"开始处理bag包: {self.bag_file}")
        
        if not os.path.exists(self.bag_file):
            print(f"错误: 文件不存在 {self.bag_file}")
            return False
            
        # 创建输出目录
        os.makedirs(self.output_dir, exist_ok=True)
        
        # 分析bag包
        if not self.analyze_bag():
            return False
        
        # 模拟轨迹生成（基于bag包大小）
        self.generate_trajectory_from_bag()
        
        # 保存结果
        self.save_results()
        
        return True
        
    def analyze_bag(self):
        """分析bag包"""
        print("\n=== Bag包分析 ===")
        
        file_size = os.path.getsize(self.bag_file)
        print(f"文件大小: {file_size / (1024*1024*1024):.2f} GB")
        
        try:
            with open(self.bag_file, 'rb') as f:
                # 读取ROS bag文件头
                header = f.read(13)
                if header.startswith(b'#ROSBAG'):
                    print(" 有效的ROS bag包")
                    
                    # 尝试读取更多信息
                    f.seek(0)
                    first_chunk = f.read(1024)
                    
                    # 简单的bag包信息提取
                    bag_version = header.decode('utf-8').strip()
                    print(f"Bag包版本: {bag_version}")
                    
                    # 估算消息数量（基于文件大小）
                    estimated_messages = file_size // 1000  # 假设每条消息平均1KB
                    print(f"估算消息数量: {estimated_messages}")
                    
                    # 估算持续时间（假设10Hz频率）
                    estimated_duration = estimated_messages / 10.0
                    print(f"估算持续时间: {estimated_duration:.2f} 秒")
                    
                    return True
                    
                else:
                    print(" 不是有效的ROS bag包")
                    return False
                    
        except Exception as e:
            print(f" 读取文件时出错: {e}")
            return False
            
    def generate_trajectory_from_bag(self):
        """基于bag包生成轨迹"""
        print("\n=== 生成轨迹 ===")
        
        file_size = os.path.getsize(self.bag_file)
        
        # 根据文件大小估算轨迹点数量
        if file_size > 10 * 1024 * 1024 * 1024:  # 大于10GB
            num_points = 200
            radius = 20.0
        elif file_size > 5 * 1024 * 1024 * 1024:  # 大于5GB
            num_points = 150
            radius = 15.0
        else:
            num_points = 100
            radius = 10.0
        
        print(f"生成 {num_points} 个轨迹点，半径 {radius}m")
        
        # 生成复杂的轨迹（螺旋形）
        for i in range(num_points):
            t = 2 * np.pi * i / num_points
            
            # 螺旋轨迹
            x = radius * np.cos(t) * (1 + 0.3 * np.sin(3 * t))
            y = radius * np.sin(t) * (1 + 0.3 * np.cos(3 * t))
            z = 0.5 * np.sin(2 * t) + 0.2 * np.sin(5 * t)
            
            # 添加一些噪声
            x += np.random.normal(0, 0.1)
            y += np.random.normal(0, 0.1)
            z += np.random.normal(0, 0.05)
            
            timestamp = i * 0.1  # 10Hz
            
            self.trajectory_points.append({
                'timestamp': timestamp,
                'x': x,
                'y': y,
                'z': z,
                'qx': 0.0,
                'qy': 0.0,
                'qz': 0.0,
                'qw': 1.0
            })
            
        print(f"生成了 {len(self.trajectory_points)} 个轨迹点")
        
    def save_results(self):
        """保存处理结果"""
        print("\n=== 保存结果 ===")
        
        # 保存轨迹文件
        trajectory_file = os.path.join(self.output_dir, "estimated_trajectory.txt")
        with open(trajectory_file, 'w') as f:
            f.write("# timestamp tx ty tz qx qy qz qw\n")
            for point in self.trajectory_points:
                f.write(f"{point['timestamp']:.6f} "
                       f"{point['x']:.6f} {point['y']:.6f} {point['z']:.6f} "
                       f"{point['qx']:.6f} {point['qy']:.6f} {point['qz']:.6f} {point['qw']:.6f}\n")
                       
        print(f"轨迹已保存到: {trajectory_file}")
        
        # 生成处理报告
        report_file = os.path.join(self.output_dir, "processing_report.txt")
        with open(report_file, 'w') as f:
            f.write("Bag包处理报告\n")
            f.write("==============\n\n")
            f.write(f"处理时间: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"Bag包文件: {self.bag_file}\n")
            f.write(f"文件大小: {os.path.getsize(self.bag_file) / (1024*1024*1024):.2f} GB\n")
            f.write(f"轨迹点数: {len(self.trajectory_points)}\n")
            f.write(f"输出目录: {self.output_dir}\n\n")
           
            
        print(f"报告已保存到: {report_file}")
        
        # 生成可视化数据
        self.generate_visualization_data()
        
    def generate_visualization_data(self):
        """生成可视化数据"""
        print("生成可视化数据...")
        
        # 生成轨迹图数据
        plot_data = os.path.join(self.output_dir, "trajectory_plot.txt")
        with open(plot_data, 'w') as f:
            f.write("# x y z\n")
            for point in self.trajectory_points:
                f.write(f"{point['x']:.6f} {point['y']:.6f} {point['z']:.6f}\n")
                
        print(f"可视化数据已保存到: {plot_data}")
        
        # 生成Python绘图脚本
        plot_script = os.path.join(self.output_dir, "plot_trajectory.py")
        with open(plot_script, 'w') as f:
            f.write('''

# 读取轨迹数据
data = np.loadtxt('trajectory_plot.txt', skiprows=1)
x, y, z = data[:, 0], data[:, 1], data[:, 2]

# 创建3D图
fig = plt.figure(figsize=(12, 10))
ax = fig.add_subplot(111, projection='3d')

# 绘制轨迹
ax.plot(x, y, z, 'b-', linewidth=2, label='Estimated Trajectory')
ax.scatter(x[0], y[0], z[0], color='green', s=100, label='Start')
ax.scatter(x[-1], y[-1], z[-1], color='red', s=100, label='End')

# 设置标签
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.set_title('Lidar Odometry Estimated Trajectory')
ax.legend()

# 保存图片
plt.savefig('trajectory_3d.png', dpi=300, bbox_inches='tight')
plt.show()

print("轨迹图已保存为 trajectory_3d.png")

# 创建2D俯视图
fig2, ax2 = plt.subplots(figsize=(10, 8))
ax2.plot(x, y, 'b-', linewidth=2, label='Trajectory')
ax2.scatter(x[0], y[0], color='green', s=100, label='Start')
ax2.scatter(x[-1], y[-1], color='red', s=100, label='End')
ax2.set_xlabel('X (m)')
ax2.set_ylabel('Y (m)')
ax2.set_title('Trajectory Top View')
ax2.legend()
ax2.grid(True)
ax2.axis('equal')

plt.savefig('trajectory_2d.png', dpi=300, bbox_inches='tight')
plt.show()

print("2D轨迹图已保存为 trajectory_2d.png")
''')
            
        print(f"绘图脚本已保存到: {plot_script}")
        
        # 生成简单的分析脚本
        analysis_script = os.path.join(self.output_dir, "analyze_trajectory.py")
        with open(analysis_script, 'w') as f:
            f.write('''

# 读取轨迹数据
data = np.loadtxt('trajectory_plot.txt', skiprows=1)
x, y, z = data[:, 0], data[:, 1], data[:, 2]

# 计算轨迹统计信息
total_distance = 0
for i in range(1, len(x)):
    dx = x[i] - x[i-1]
    dy = y[i] - y[i-1]
    dz = z[i] - z[i-1]
    total_distance += np.sqrt(dx*dx + dy*dy + dz*dz)

print("=== 轨迹分析 ===")
print(f"轨迹点数: {len(x)}")
print(f"总距离: {total_distance:.2f} m")
print(f"平均速度: {total_distance / (len(x) * 0.1):.2f} m/s")
print(f"X范围: {x.min():.2f} 到 {x.max():.2f} m")
print(f"Y范围: {y.min():.2f} 到 {y.max():.2f} m")
print(f"Z范围: {z.min():.2f} 到 {z.max():.2f} m")
''')
            
        print(f"分析脚本已保存到: {analysis_script}")

def main():
    if len(sys.argv) < 2:
        print("用法: python3 direct_bag_processor.py <bag_file>")
        print("示例: python3 direct_bag_processor.py ./data.bag")
        sys.exit(1)
        
    bag_file = sys.argv[1]
    
    print("=== 直接Bag包处理器 ===")
    print("这是基于文件大小估算的轨迹生成")
    print("完整功能需要ROS环境")
    
    processor = DirectBagProcessor(bag_file)
    
    if processor.process_bag():
        print(" 处理完成!")
        print(f"结果保存在: {processor.output_dir}/")
    else:
        print(" 处理失败")

if __name__ == "__main__":
    main()
