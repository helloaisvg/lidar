#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt

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
