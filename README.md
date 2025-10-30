# 激光雷达里程计项目

##  项目概述

本项目是一个完整的激光雷达里程计系统，实现了NDT、ICP和GN-ICP三种配准算法，支持KITTI数据集处理和ROS bag包分析。项目包含完整的代码实现、视频录制、地图保存、轨迹评估等功能。

##  项目完成情况

### 任务1: 配准算法实现 
- **NDT配准**: 使用PCL库实现正态分布变换配准
- **ICP配准**: 使用PCL库实现标准迭代最近点配准算法
- **GN-ICP配准**: 手写高斯牛顿ICP配准算法
- **代码位置**: `src/ndt_registration.cpp`, `src/icp_registration.cpp`, `src/gn_icp_registration.cpp`

### 任务2: KITTI里程计和视频录制 
- **KITTI里程计**: 基于KITTI数据集的NDT、ICP和GN-ICP里程计
- **配准方法**: 支持NDT（PCL）、ICP（PCL标准）、GN-ICP（手写）三种方法
- **视频录制**: 生成MP4格式的里程计运行视频（OpenCV）
- **地图截图**: 保存全局地图PNG截图
- **可执行文件**: `build/kitti_simple`, `build/kitti_enhanced_odometry`
- **代码位置**: `src/kitti_simple.cpp`, `src/kitti_enhanced_odometry.cpp`, `src/lidar_odometry_enhanced.cpp`

### 任务3: ROS节点和EVO评估 
- **ROS2节点**: 实现ROS2里程计节点，订阅点云话题，发布位姿/里程计/路径/TF，保存轨迹（TUM格式）
- **EVO评估**: 使用EVO工具进行轨迹评估和可视化
- **可执行文件**: `build/ros_odometry_node`
- **代码位置**: `src/ros_odometry_node.cpp`, `scripts/evaluate_with_evo.sh`, `scripts/evaluate_simple.py`

### 任务4: Bag包处理 
- **真实数据提取**:  使用rosbags库解析ROS 1 bag文件中的PointCloud2消息
- **点云处理**:  每帧提取12-13万个真实点云坐标
- **轨迹生成**:  使用点云质心估算轨迹（**非真实里程计，仅供参考**）
- **可视化工具**:  生成2D/3D轨迹图、速度分析和统计报告
- **代码位置**: `scripts/direct_bag_processor.py`


##  项目结构

```
lidar/
├── src/                          # 源代码文件
│   ├── ndt_registration.cpp      # NDT配准算法
│   ├── icp_registration.cpp      # ICP配准算法（PCL标准）
│   ├── gn_icp_registration.cpp   # GN-ICP配准算法（手写）
│   ├── lidar_odometry.cpp        # 里程计核心实现
│   ├── kitti_simple.cpp          # KITTI简单入口
│   ├── kitti_enhanced_odometry.cpp # KITTI增强版（含视频录制）
│   ├── ros_odometry_node.cpp     # ROS2节点
│   └── ...
├── include/lidar_odometry/       # 头文件
├── scripts/                      # 脚本文件 (5个)
│   ├── direct_bag_processor.py   # Bag包处理器
│   ├── evaluate_with_evo.sh      # EVO评估脚本
│   ├── show_results.py           # 结果展示脚本
│   ├── simple_trajectory_evaluator.py # 轨迹评估脚本
│   └── view_pointcloud.py       # 点云查看器
├── results/                      # 结果文件 
│   ├── screenshots/            # 截图文件
│   ├── videos/                 # 视频文件
│   └── visualizations/         # 可视化文件
├── evo_evaluation_results/       # EVO评估结果
│   ├── ndt/                      # NDT评估结果 (8个文件)
│   └── gn_icp/                   # GN-ICP评估结果 (8个文件)
├── data_odometry_velodyne/       # KITTI数据集
├── build/                       # 编译输出
│   ├── kitti_simple             # KITTI简单可执行
│   ├── kitti_enhanced_odometry  # KITTI增强可执行
│   └── ros_odometry_node        # ROS2节点可执行
├── evo_evaluation/              # EVOeles结果目录
├── .evoenv/                     # EVO虚拟环境
├── results/                     # 结果文件目录（可选）


```

##  快速开始

### 1. 编译项目
```bash
cd /home/sss/lidar/lidar
mkdir -p build && cd build
cmake ..
cmake --build . -j4
```

编译完成后，在`build/`目录下会生成以下可执行文件：
- `kitti_simple` - 简单KITTI里程计（快速测试）
- `kitti_enhanced_odometry` - 增强版KITTI里程计（含视频录制）
- `ros_odometry_node` - ROS2里程计节点

### 2. 运行KITTI里程计
```bash
# 简单版NDT里程计（处理前10帧）
./build/kitti_simple /home/sss/lidar/lidar/data_odometry_velodyne/dataset 0 ndt

# 简单版ICP里程计（PCL标准）
./build/kitti_simple /home/sss/lidar/lidar/data_odometry_velodyne/dataset 0 icp

# 简单版GN-ICP里程计
./build/kitti_simple /home/sss/lidar/lidar/data_odometry_velodyne/dataset 0 gn_icp

# 增强版里程计（包含视频录制、地图保存，处理前10帧）
./build/kitti_enhanced_odometry /home/sss/lidar/lidar/data_odometry_velodyne/dataset 0 ndt
./build/kitti_enhanced_odometry /home/sss/lidar/lidar/data_odometry_velodyne/dataset 0 icp
./build/kitti_enhanced_odometry /home/sss/lidar/lidar/data_odometry_velodyne/dataset 0 gn_icp
```

**输出文件**（生成在项目根目录）：
- `ndt_kitti_0_trajectory.txt` / `icp_kitti_0_trajectory.txt` / `gn_icp_kitti_0_trajectory.txt` - 轨迹文件（TUM格式）
- `*_kitti_0_global_map.pcd` - 全局地图点云
- `*_kitti_0_global_map_screenshot.png` - 地图截图
- `*_kitti_0_odometry_video.mp4` - 里程计运行视频（仅增强版）

### 3. 处理Bag包 
```bash
# 处理bag包提取点云数据（支持ROS 1格式）
python3 scripts/direct_bag_processor.py ./data.bag ndt

# 注意：目前只提取点云数据，轨迹使用质心估算
# 查看结果
cd bag_processing_results && python3 analyze_trajectory.py

# EVO可视化（推荐）
export PATH="$HOME/.local/bin:$PATH"
evo_traj tum bag_processing_results/estimated_trajectory.txt --plot --save_plot evo_evaluation_results/bag_plot
```



### 4. ROS2节点运行
```bash
# 需要先source ROS2环境
source /opt/ros/jazzy/setup.bash

# 运行ROS2里程计节点
./build/ros_odometry_node

# 节点参数：
# - pointcloud_topic: /velodyne_points (默认)
# - registration_method: ndt, icp, 或 gn_icp (默认: ndt)
# - trajectory_output_file: ros_odometry_trajectory.txt (默认)
```

### 5. EVO评估
```bash
# 激活EVO虚拟环境
source .evoenv/bin/activate

# 轨迹对齐和对比（无真值时）
./.evoenv/bin/evo_traj tum ndt_kitti_0_trajectory.txt icp_kitti_0_trajectory.txt gn_icp_kitti_0_trajectory.txt \
  --ref ndt_kitti_0_trajectory.txt --align --save_table evo_evaluation/comparison_stats.txt

# 或使用简化评估脚本（无依赖）
python3 scripts/evaluate_simple.py ndt_kitti_0_trajectory.txt icp_kitti_0_trajectory.txt gn_icp_kitti_0_trajectory.txt evo_evaluation
```

### 6. 查看结果
```bash
# 查看点云文件（示例）
python3 scripts/view_pointcloud.py ndt_kitti_0_global_map.pcd

# 查看生成的文件
ls -lh ndt_kitti_0_* icp_kitti_0_* gn_icp_kitti_0_* *.tum
```

##  生成的结果文件

###  轨迹文件（项目根目录）
- `ndt_kitti_0_trajectory.txt` - NDT轨迹(TUM格式)
- `icp_kitti_0_trajectory.txt` - ICP轨迹(TUM格式)
- `gn_icp_kitti_0_trajectory.txt` - GN-ICP轨迹(TUM格式)
- `ros_odometry_trajectory.txt` - ROS2节点输出的轨迹（如运行）

###  点云文件（项目根目录）
- `ndt_kitti_0_global_map.pcd` - NDT全局地图点云
- `icp_kitti_0_global_map.pcd` - ICP全局地图点云
- `gn_icp_kitti_0_global_map.pcd` - GN-ICP全局地图点云

###  截图文件（项目根目录）
- `ndt_kitti_0_global_map_screenshot.png` - NDT全局地图截图
- `icp_kitti_0_global_map_screenshot.png` - ICP全局地图截图
- `gn_icp_kitti_0_global_map_screenshot.png` - GN-ICP全局地图截图

###  视频文件（项目根目录，简单版不生成，仅增强版生成）
- `ndt_kitti_0_odometry_video.mp4` - NDT里程计运行视频
- `icp_kitti_0_odometry_video.mp4` - ICP里程计运行视频
- `gn_icp_kitti_0_odometry_video.mp4` - GN-ICP里程计运行视频

###  EVO评估结果 (`evo_evaluation/`)
- `evo_evaluation/relative_simple_metrics.txt` - 相对误差指标（RMSE/MAE/MAX）
- `evo_evaluation/comparison_stats.txt` - EVO对齐统计表（NDT/ICP/GN-ICP对比）

##  环境要求

- **CMake 3.16+**
- **PCL (Point Cloud Library) 1.8+**
- **Eigen3 3.3+**
- **OpenCV 4.0+**
- **Python 3.8+**
- **EVO** 

##  性能统计（前10帧示例）

### 轨迹统计
- **处理帧数**: 10帧（可调整）
- **成功帧数**: 10帧（成功率100%）
- **NDT平均处理时间**: ~35秒/帧（包含预处理和配准）
- **ICP平均处理时间**: ~2秒/帧（PCL标准实现，较快）
- **GN-ICP平均处理时间**: ~40秒/帧

### 点云统计
- **单帧点云数**: 约55,000-60,000点（过滤后）
- **全局地图点数**: 约20,000-30,000点（累积后）
- **地图文件大小**: ~500KB-1MB (PCD格式)

##  主要功能

1. **配准算法**: NDT（PCL）、ICP（PCL标准）和GN-ICP（手写）三种配准方法
2. **KITTI里程计**: 基于KITTI数据集的激光雷达里程计（NDT/ICP/GN-ICP）
3. **视频录制**: 自动录制里程计运行过程（MP4格式，增强版）
4. **地图保存**: 保存全局地图为PCD格式和PNG截图
5. **轨迹评估**: 使用EVO工具进行轨迹对齐和对比分析
6. **ROS2节点**: ROS2里程计节点，支持在线处理和轨迹保存
7. **Bag包处理**: 提取真实点云数据（轨迹使用质心估算，仅供参考）


##  License

本项目仅供学习和研究使用。

