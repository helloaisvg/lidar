#!/usr/bin/env python3
import numpy as np

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
