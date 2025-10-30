# 激光雷达前端里程计项目报告

## 1. 人员分工

- 独立完成：
  - 需求分析与技术路线制定
  - 算法实现：PCL-NDT、PCL-ICP（标准）、手写 GN-ICP 三种配准方法
  - 基于 KITTI 数据集的前端里程计实现与运行视频录制（NDT/ICP/GN-ICP）
  - 全局地图保存与截图、轨迹导出（TUM 格式）
  - ROS2 节点编写（里程计轨迹保存）与参数化启动
  - EVO 评估（有真值）与 EVO 可视化（无真值 Bag）
  - Bag 包真实点云数据解析与可视化流水线
  - 文档撰写与结果归档（README、指南、总结、报告）


## 2. 需求分析（功能与预期）

### 2.1 功能清单
1) 配准算法：
   - 使用 PCL 库补充完成 NDT 配准，实现两帧点云配准；
   - 使用 PCL 库实现标准 ICP 配准，实现两帧点云配准；
   - 调用手写的 GN-ICP 实现配准（作为与 PCL-NDT/ICP 的对比）。
2) 前端里程计（KITTI）：
   - 使用（1）中的配准方法实现基于 KITTI 数据集的前端里程计（NDT/ICP/GN-ICP）；
   - 对不同配准方法，录制运行视频，并截图展示最终全局地图；
   - 导出 TUM 轨迹。
3) ROS 节点与 EVO 评估：
   - 编写 ROS2 节点，实现里程计轨迹（TUM）保存；
   - 使用 EVO 工具对比不同配准方法的里程计与真值差异（ATE/RPE、轨迹图等）。
4) Bag 包处理与可视化：
   - 录制小车 Lidar 数据 Bag 包并运行里程计；
   - 无真值情况下，使用 EVO 对里程计轨迹做可视化展示与定性分析。

### 2.2 预期效果
- 两帧配准：对中/小初值偏差具有收敛能力（NDT 稳健、ICP 收敛速度快、GN-ICP 精度高）。
- KITTI 前端：输出连续平滑的轨迹与清晰的全局地图；不同方法在速度/精度/鲁棒性上可对比；
- ROS2 与评估：可在线保存 TUM 轨迹；EVO 输出可视化与误差指标（有真值）；
- Bag 可视化：在无真值时提供轨迹与姿态/速度曲线图，便于快速诊断。


## 3. 程序设计（模块、类/函数、相互关系、算法流程）

### 3.1 代码结构与关键文件
- 配准算法（Registration）
  - `src/ndt_registration.cpp`：PCL NDT 配准实现（正态分布变换）。
  - `src/icp_registration.cpp`：PCL 标准 ICP 配准实现（迭代最近点算法）。
  - `src/gn_icp_registration.cpp`：手写 GN-ICP（高斯牛顿迭代，点-点/点-面残差、可选鲁棒核）。
  - 对应头文件：`include/lidar_odometry/*.h`（接口声明、参数结构）。
- 前端里程计（Odometry）
  - `src/kitti_simple.cpp`：KITTI简单入口（快速测试，处理前10帧）。
  - `src/kitti_enhanced_odometry.cpp`：增强版（含视频录制、地图保存、统计）。
  - `src/lidar_odometry.cpp`、`src/lidar_odometry_enhanced.cpp`：里程计核心封装。
- 传感器/工具
  - `src/imu_integration.cpp`：IMU 积分与运动补偿（若启用）。
  - `src/trajectory_evaluator.cpp`：轨迹评估辅助（统计、文件读写）。
  - `src/ros_odometry_node.cpp`：ROS2 节点（订阅 `sensor_msgs/msg::PointCloud2`、发布位姿/里程计/路径/TF、保存 TUM 轨迹）。
- 启动与可视化
  - `launch/lidar_odometry.launch`：节点/参数启动。
  - `config/lidar_odometry.rviz`：RViz 配置（点云、轨迹、TF）。
- 评估与工具脚本
  - `scripts/evaluate_with_evo.sh`、`scripts/evaluate_simple.py`、`scripts/evaluate_relative.py`、`scripts/simple_trajectory_evaluator.py`
  - `scripts/show_results.py`、`scripts/view_pointcloud.py`
- Bag 数据处理
  - `scripts/direct_bag_processor.py`：解析 ROS 1 Bag（`sensor_msgs/PointCloud2`），提取真实点、导出 TUM、统计与可视化数据。

### 3.2 模块关系与数据流
- KITTI 前端数据流：
  1) 读取 KITTI 二进制激光雷达帧 → 2) 预处理（下采样/去噪/裁剪） → 3) 选择配准器（NDT/ICP/GN-ICP） → 4) 估计位姿增量 ΔT → 5) 累积位姿 T_k = T_{k-1} · ΔT → 6) 关键帧策略与全局地图更新 → 7) 输出 TUM 轨迹、保存地图、录制视频/截图。
- ROS 节点：
  - 订阅 `PointCloud2` → 转换为内部点云 → 调用前端处理（与 KITTI 相同接口）→ 发布/保存 TUM 轨迹 → 可在 RViz 查看。
- EVO 评估/可视化：
  - 有真值：`evo_traj tum est.tum --ref gt.tum ...` 输出 ATE/RPE、轨迹叠加图等；
  - 无真值：`evo_traj tum est.tum --plot --save_plot ...` 生成轨迹/姿态/速度等图。
- Bag 处理：
  - 读取 `data.bag` → 解析 `sensor_msgs/PointCloud2` → 抽取 x/y/z → 统计/轨迹生成（当前为质心轨迹，便于可视化）→ 输出 TUM 与评估图片。

### 3.3 关键类/函数
- Registration 接口：`bool align(PointCloud src, PointCloud tgt, Pose& delta)`
  - NDT：构建体素-高斯模型，使用内部优化器迭代求解；
   - ICP：对应搜索（KD-Tree）→ 计算对应关系 → 求解最优变换矩阵 → 迭代直到收敛；
  - GN-ICP：对应搜索（KD-Tree/法线）→ 线性化 → 构建法方程 H·δ = -g → 求解 δ → 更新位姿，收敛判定。
- Odometry 核心：`processFrame(pointCloud) -> Pose`
  - 管线：预处理 → 选择参考（局部地图/上一帧）→ 调用 Registration → 更新里程计状态机 → 触发可视化/保存事件。
- Trajectory I/O：`writeTum(file, t, tx, ty, tz, qx, qy, qz, qw)`

### 3.4 算法流程
- PCL-NDT：
  - 体素化目标点云 → 每体素估计均值/协方差 → 在位姿参数上进行似然最大化迭代 → 输出最优位姿增量。
- PCL-ICP：
  - 源点云变换 → KD-Tree 最近邻搜索 → 计算对应关系 → 求解最优刚体变换（SVD）→ 迭代直到收敛或达到最大迭代次数。
- 手写 GN-ICP：
  - 找对应（点-点/点-面）；计算残差 r 和雅可比 J；
  - 累积 H = Σ JᵀJ，g = Σ Jᵀr；解 δ = -H⁻¹g；
  - 更新位姿（李代数 SE(3) 指数映射）；收敛判定（|δ|、残差下降、迭代上限）。
- 前端里程计：
  - 初值（常速/零增量/IMU）；
  - 帧间配准 → 位姿累计 → 关键帧插入策略（距离/角度阈值）→ 局部地图维护（下采样、移窗）→ 导出。

### 3.5 参数与配置
- 预处理：体素下采样 voxel=0.1–0.2 m；半径裁剪 1–100 m；min_points 1000。
- NDT：分辨率 1.0 m；最大迭代 35；epsilon 1e-4；step_size 0.1。
- ICP：最大对应距离 1.0 m；最大迭代 50；变换收敛阈值 1e-6；适应度阈值 1e-6。
- GN-ICP：最近邻半径 1.0–2.0 m；最大配对距离 0.5 m；最大迭代 50；核函数 Huber/Cauchy；早停阈值 1e-6。
- 关键帧：平移阈值 1–3 m；旋转阈值 5–10°；地图下采样 0.2–0.5 m。


## 4. 程序效果展示

### 4.1 KITTI 前端里程计
- 运行入口（示例）：
```bash
# 简单版（快速测试，处理前10帧）
./build/kitti_simple /home/sss/lidar/lidar/data_odometry_velodyne/dataset 0 ndt
./build/kitti_simple /home/sss/lidar/lidar/data_odometry_velodyne/dataset 0 icp
./build/kitti_simple /home/sss/lidar/lidar/data_odometry_velodyne/dataset 0 gn_icp

# 增强版（含视频录制/地图保存，处理前10帧）
./build/kitti_enhanced_odometry /home/sss/lidar/lidar/data_odometry_velodyne/dataset 0 ndt
./build/kitti_enhanced_odometry /home/sss/lidar/lidar/data_odometry_velodyne/dataset 0 icp
./build/kitti_enhanced_odometry /home/sss/lidar/lidar/data_odometry_velodyne/dataset 0 gn_icp
```
- 产出与展示（生成在项目根目录）：
  - 视频：`ndt_kitti_0_odometry_video.mp4`、`icp_kitti_0_odometry_video.mp4`、`gn_icp_kitti_0_odometry_video.mp4`
  - 地图截图：`ndt_kitti_0_global_map_screenshot.png`、`icp_kitti_0_global_map_screenshot.png`、`gn_icp_kitti_0_global_map_screenshot.png`
  - 地图点云：`ndt_kitti_0_global_map.pcd`、`icp_kitti_0_global_map.pcd`、`gn_icp_kitti_0_global_map.pcd`
  - 轨迹（TUM）：`ndt_kitti_0_trajectory.txt`、`icp_kitti_0_trajectory.txt`、`gn_icp_kitti_0_trajectory.txt`



### 4.2 ROS 节点与 EVO 评估（有真值）
- 运行与评估（示例）：
```bash
# 启动 ROS2 里程计节点
source /opt/ros/jazzy/setup.bash
./build/ros_odometry_node

# 节点参数（可通过ROS2参数系统设置）：
# - registration_method: ndt, icp, 或 gn_icp (默认: ndt)
# - pointcloud_topic: /velodyne_points (默认)
# - trajectory_output_file: ros_odometry_trajectory.txt (默认)

# 使用 EVO 与真值对齐评估（示意）
source .evoenv/bin/activate
evo_traj tum ndt_kitti_0_trajectory.txt icp_kitti_0_trajectory.txt gn_icp_kitti_0_trajectory.txt \
  --ref path/to/kitti_gt.tum --align --save_table evo_evaluation/comparison_stats.txt
```
- 产出：`evo_evaluation_results/ndt/`、`evo_evaluation_results/icp/`、`evo_evaluation_results/gn_icp/` 下的评估图与统计文本（ATE/RPE、轨迹叠加等）。



### 4.3 Bag 包（无真值，可视化）
- 数据与规模（已验证）：
  - `data.bag`（约 14.49 GB），Topic `/kitti/velo/pointcloud`，类型 `sensor_msgs/PointCloud2`；
  - 1000 帧，约 63k–127k 点/帧，总计约 8,000 万+ 点；
  - 输出轨迹 1000 点，TUM 格式；总距离约 202.75 m；平均速度约 2.03 m/s。
- 运行与展示：
```bash
# 处理 bag 包并导出 TUM（当前为质心轨迹，用于可视化快速验证）
python3 scripts/direct_bag_processor.py data.bag ndt

# EVO 可视化
bash scripts/evaluate_with_evo.sh
```
- 可视化产出：`evo_evaluation_results/trajectory_plot_trajectories.png`、`trajectory_plot_xyz.png`、`trajectory_plot_rpy.png`、`trajectory_plot_speeds.png`


### 4.4 性能与资源
- 环境：CMake 3.16+，PCL 1.8+，Eigen 3.3+，OpenCV 4.0+，Python 3.8+，ROS2；可选 EVO。
- 运行时长（参考，前10帧处理）：
  - NDT：平均处理时间 ~35秒/帧（包含预处理和配准）
  - ICP：平均处理时间 ~2秒/帧（PCL标准实现，较快）
  - GN-ICP：平均处理时间 ~40秒/帧
  - Bag 解析（1000 帧）约 1 分钟级。


## 5. 总结分析（效果 vs 理想、差异原因、改进方向）

### 5.1 实现效果 vs 理想目标
- 已达成：
  - PCL-NDT、PCL-ICP（标准）与手写 GN-ICP 三种配准方法的两帧配准；
  - 基于 KITTI 的 NDT/ICP/GN-ICP 前端里程计算法与结果产出（视频、地图、轨迹）；
  - ROS2 节点轨迹保存（支持三种配准方法切换）；
  - EVO 评估（有真值）与可视化（无真值 Bag）；
  - Bag 真实点云解析与轨迹可视化全流程。
- 与理想的差距：
  - ICP 需要良好初值，对初始位姿敏感，初值偏差大时可能收敛到局部最优；
  - GN-ICP 在退化/稀疏/动态场景存在收敛慢或局部极值风险；
  - Bag 无真值场景无法量化 ATE/RPE；
  - 参数需要场景自适应（体素分辨率、阈值、鲁棒核权重等）。

### 5.2 原因分析
- 传感器与场景几何退化（平面走廊/远距稀疏）弱化约束，ICP 类方法稳定性下降；
- ICP 对初始位姿要求较高，初值偏差大时可能收敛到局部最优；
- 未充分建模运动畸变与外点，对高速转弯/加速段敏感；
- 纯前端难以保证全局一致性（若无回环/后端优化）；
- 无真值 Bag 只能做定性可视化，难以进行闭环定量验证。

### 5.3 改进方向
- 前端增强：
  - 融合 IMU（去畸变、初值预测），为 ICP 提供良好初值；
  - 动态对应关系筛选与鲁棒核；
  - 自适应体素与阈值策略（基于局部几何复杂度）；
  - 多尺度配准策略（粗到细）。
- 后端优化：
  - 引入回环检测与位姿图优化，提升全局一致性；
  - 跨方法融合（NDT 提供初值，ICP/GN-ICP 精配准）。
- Bag 真值补充：
  - 同步其他传感器（里程计/GNSS/视觉）以构造参考真值；
  - 或通过局部标定场景获取片段真值，进行分段验证。
- 工程集成：
  - 通过 pybind11 将 C++ 前端嵌入 Python Bag 流水线，实现“真实配准驱动”的 Bag 轨迹（替代质心轨迹）。


## 6. 复现实验与附录

### 6.1 构建与运行（示例）
```bash
cd /home/sss/lidar/lidar
mkdir -p build && cd build
cmake ..
make -j8
```

### 6.2 KITTI 运行
```bash
# 简单版（NDT / ICP / GN-ICP）
./build/kitti_simple /home/sss/lidar/lidar/data_odometry_velodyne/dataset 0 ndt
./build/kitti_simple /home/sss/lidar/lidar/data_odometry_velodyne/dataset 0 icp
./build/kitti_simple /home/sss/lidar/lidar/data_odometry_velodyne/dataset 0 gn_icp

# 增强版（含视频录制、地图保存）
./build/kitti_enhanced_odometry /home/sss/lidar/lidar/data_odometry_velodyne/dataset 0 ndt
./build/kitti_enhanced_odometry /home/sss/lidar/lidar/data_odometry_velodyne/dataset 0 icp
./build/kitti_enhanced_odometry /home/sss/lidar/lidar/data_odometry_velodyne/dataset 0 gn_icp
```

### 6.3 ROS2 节点与评估
```bash
# 启动 ROS2 节点
source /opt/ros/jazzy/setup.bash
./build/ros_odometry_node

# EVO 评估（三种方法对比）
source .evoenv/bin/activate
evo_traj tum ndt_kitti_0_trajectory.txt icp_kitti_0_trajectory.txt gn_icp_kitti_0_trajectory.txt \
  --ref path/to/kitti_gt.tum --align --save_table evo_evaluation/comparison_stats.txt
```

### 6.4 Bag 处理与可视化
```bash
python3 scripts/direct_bag_processor.py data.bag ndt
bash scripts/evaluate_with_evo.sh
```







