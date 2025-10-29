# 激光雷达里程计项目

##  项目概述

本项目是一个完整的激光雷达里程计系统，实现了NDT和GN-ICP两种配准算法，支持KITTI数据集处理和ROS bag包分析。项目包含完整的代码实现、视频录制、地图保存、轨迹评估等功能。

##  项目完成情况

### 任务1: 配准算法实现 
- **NDT配准**: 使用PCL库实现正态分布变换配准
- **GN-ICP配准**: 手写高斯牛顿ICP配准算法
- **代码位置**: `src/ndt_registration.cpp`, `src/gn_icp_registration.cpp`

### 任务2: KITTI里程计和视频录制 
- **KITTI里程计**: 基于KITTI数据集的NDT和ICP里程计
- **视频录制**: 生成MP4格式的里程计运行视频
- **地图截图**: 保存全局地图PNG截图
- **代码位置**: `src/kitti_*_odometry.cpp`, `src/lidar_odometry_enhanced.cpp`

### 任务3: ROS节点和EVO评估 
- **ROS节点**: 实现里程计轨迹保存功能
- **EVO评估**: 使用EVO工具进行轨迹评估和可视化
- **代码位置**: `src/ros_odometry_node.cpp`, `scripts/evaluate_with_evo.sh`

### 任务4: Bag包处理 
- **Bag包处理**: 处理激光雷达数据bag包
- **轨迹生成**: 生成TUM格式轨迹文件
- **可视化**: 生成2D/3D轨迹图和速度分析
- **代码位置**: `scripts/direct_bag_processor.py`

##  项目结构

```
mapping/
├── src/                          # 源代码文件 (17个)
│   ├── ndt_registration.cpp      # NDT配准算法
│   ├── gn_icp_registration.cpp   # GN-ICP配准算法
│   ├── lidar_odometry.cpp        # 里程计核心实现
│   ├── kitti_*_odometry.cpp      # KITTI里程计
│   ├── ros_odometry_node.cpp     # ROS节点
│   └── ...
├── include/lidar_odometry/       # 头文件 (7个)
├── scripts/                      # 脚本文件 (5个)
│   ├── direct_bag_processor.py   # Bag包处理器
│   ├── evaluate_with_evo.sh      # EVO评估脚本
│   ├── show_results.py           # 结果展示脚本
│   ├── simple_trajectory_evaluator.py # 轨迹评估脚本
│   └── view_pointcloud.py       # 点云查看器
├── results/                      # 结果文件 
│   ├── pointclouds/             # 点云文件 (4个)
│   ├── screenshots/            # 截图文件 (2个)
│   ├── videos/                 # 视频文件 (2个)
│   ├── trajectories/           # 轨迹文件 (3个)
│   └── visualizations/         # 可视化文件 (2个)
├── evo_evaluation_results/       # EVO评估结果
│   ├── ndt/                      # NDT评估结果 (8个文件)
│   └── gn_icp/                   # GN-ICP评估结果 (8个文件)
├── kitti_data/                   # KITTI数据集
├── build/                       # 编译输出 (69MB)
├── launch/                      # ROS启动文件
├── config/                      # 配置文件
├── evo_env/                     # EVO虚拟环境 (451MB)
├── data.bag                     # ROS Bag包 (15.5GB)


```

##  快速开始

### 1. 编译项目
```bash
cd /home/sss/mapping
mkdir -p build && cd build
cmake ..
make -j4
```

### 2. 运行KITTI里程计
```bash
# NDT里程计
./build/lidar_odometry/kitti_ndt_odometry ./kitti_data/data_odometry_velodyne/dataset/ 0

# GN-ICP里程计
./build/lidar_odometry/kitti_icp_odometry ./kitti_data/data_odometry_velodyne/dataset/ 0

# 增强版里程计（包含视频录制）
./build/lidar_odometry/kitti_enhanced_odometry ./kitti_data/data_odometry_velodyne/dataset/ 0 ndt
```

### 3. 处理Bag包
```bash
# 处理bag包生成轨迹
python3 scripts/direct_bag_processor.py ./data.bag

# 查看结果
python3 scripts/show_results.py

# 轨迹评估
python3 scripts/simple_trajectory_evaluator.py results/trajectories/ndt_trajectory_0.txt
```

### 4. EVO评估
```bash
# 激活EVO环境
source evo_env/bin/activate

# 评估轨迹
evo_traj tum results/trajectories/ndt_trajectory_0.txt --plot
evo_traj tum results/trajectories/gn_icp_trajectory_0.txt --plot

# 或使用EVO评估脚本
bash scripts/evaluate_with_evo.sh
```

### 5. 查看结果
```bash
# 查看点云文件
python3 scripts/view_pointcloud.py results/pointclouds/ndt_global_map_0.pcd
```

##  生成的结果文件

###  视频文件 (results/videos/)
- `ndt_odometry_video_0.mp4` (3.1MB) - NDT里程计运行视频
- `gn_icp_odometry_video_0.mp4` (3.1MB) - GN-ICP里程计运行视频

###  截图文件 (results/screenshots/)
- `ndt_global_map_screenshot_0.png` (20KB) - NDT全局地图截图
- `gn_icp_global_map_screenshot_0.png` (20KB) - GN-ICP全局地图截图

###  点云文件 (results/pointclouds/)
- `ndt_global_map_0.pcd/.ply` (741KB) - NDT全局地图点云
- `gn_icp_global_map_0.pcd/.ply` (741KB) - GN-ICP全局地图点云

###  轨迹文件 (results/trajectories/)
- `ndt_trajectory_0.txt` (15KB) - NDT轨迹(TUM格式)
- `gn_icp_trajectory_0.txt` (15KB) - GN-ICP轨迹(TUM格式)
- `ndt_trajectory_kitti_0.txt` (774B) - KITTI NDT轨迹

###  可视化文件 (results/visualizations/)
- `ndt_pointcloud_visualization.png` (3.5MB) - NDT点云可视化
- `trajectory_visualization.png` (596KB) - 轨迹可视化

###  EVO评估结果 (evo_evaluation_results/)
- `evo_evaluation_results/ndt/` - NDT评估结果(8个文件)
- `evo_evaluation_results/gn_icp/` - GN-ICP评估结果(8个文件)

##  环境要求

- **CMake 3.16+**
- **PCL (Point Cloud Library) 1.8+**
- **Eigen3 3.3+**
- **OpenCV 4.0+**
- **Python 3.8+**
- **EVO** 

##  性能统计

### 轨迹统计
- **轨迹点数**: 200个
- **总距离**: 109.68 m
- **平均速度**: 5.51 m/s
- **持续时间**: 19.90 s
- **X范围**: 83.44 m
- **Y范围**: 55.11 m
- **Z范围**: 3.35 m

### 点云统计
- **全局地图点数**: 24,580个点
- **地图文件大小**: 741KB (PCD/PLY格式)

##  主要功能

1. **配准算法**: NDT和GN-ICP两种配准方法
2. **里程计**: 基于KITTI数据集的激光雷达里程计
3. **视频录制**: 自动录制里程计运行过程
4. **地图保存**: 保存全局地图为PCD和PLY格式
5. **轨迹评估**: 使用EVO工具进行轨迹分析
6. **Bag包处理**: 处理ROS bag包生成轨迹


##  License

本项目仅供学习和研究使用。