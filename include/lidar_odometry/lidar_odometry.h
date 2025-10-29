#ifndef LIDAR_ODOMETRY_H
#define LIDAR_ODOMETRY_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <memory>
#include <vector>
#include <string>
#include <fstream>
#include <chrono>
#include <thread>

#include "ndt_registration.h"
#include "gn_icp_registration.h"

namespace lidar_odometry {

/**
 * @brief 激光雷达里程计类
 * 基于KITTI数据集实现激光雷达里程计
 */
class LidarOdometry {
public:
    using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
    using PointCloudPtr = PointCloud::Ptr;

    /**
     * @brief 配准方法枚举
     */
    enum class RegistrationMethod {
        NDT,        // NDT配准
        GN_ICP      // GN-ICP配准
    };

    /**
     * @brief 轨迹点结构
     */
    struct TrajectoryPoint {
        double timestamp;           // 时间戳
        Eigen::Vector3d position;  // 位置
        Eigen::Quaterniond orientation;  // 姿态四元数
        Eigen::Matrix4d transformation;  // 变换矩阵
        
        TrajectoryPoint() : timestamp(0.0) {}
        
        TrajectoryPoint(double t, const Eigen::Vector3d& pos, const Eigen::Quaterniond& quat)
            : timestamp(t), position(pos), orientation(quat) {
            transformation = Eigen::Matrix4d::Identity();
            transformation.block<3, 3>(0, 0) = orientation.toRotationMatrix();
            transformation.block<3, 1>(0, 3) = position;
        }
    };

    /**
     * @brief 构造函数
     * @param method 配准方法
     * @param voxel_size 体素大小
     * @param max_range 最大距离
     * @param min_range 最小距离
     */
    LidarOdometry(RegistrationMethod method = RegistrationMethod::NDT,
                  double voxel_size = 0.1,
                  double max_range = 100.0,
                  double min_range = 1.0);

    /**
     * @brief 析构函数
     */
    ~LidarOdometry() = default;

    /**
     * @brief 设置配准方法
     * @param method 配准方法
     */
    void setRegistrationMethod(RegistrationMethod method);

    /**
     * @brief 设置体素大小
     * @param voxel_size 体素大小
     */
    void setVoxelSize(double voxel_size);

    /**
     * @brief 设置距离范围
     * @param min_range 最小距离
     * @param max_range 最大距离
     */
    void setRangeFilter(double min_range, double max_range);

    /**
     * @brief 处理单帧点云
     * @param cloud 输入点云
     * @param timestamp 时间戳
     * @return 是否成功处理
     */
    bool processFrame(const PointCloudPtr& cloud, double timestamp);

    /**
     * @brief 处理KITTI数据集
     * @param data_path KITTI数据路径
     * @param sequence 序列号
     * @return 是否成功处理
     */
    bool processKITTISequence(const std::string& data_path, int sequence);

    /**
     * @brief 获取当前位姿
     * @return 当前位姿变换矩阵
     */
    Eigen::Matrix4d getCurrentPose() const;

    /**
     * @brief 获取轨迹
     * @return 轨迹点列表
     */
    const std::vector<TrajectoryPoint>& getTrajectory() const;

    /**
     * @brief 保存轨迹到文件
     * @param filename 文件名
     * @return 是否成功保存
     */
    bool saveTrajectory(const std::string& filename) const;

    /**
     * @brief 加载轨迹从文件
     * @param filename 文件名
     * @return 是否成功加载
     */
    bool loadTrajectory(const std::string& filename);

    /**
     * @brief 可视化轨迹
     * @param show_clouds 是否显示点云
     */
    void visualizeTrajectory(bool show_clouds = false);

    /**
     * @brief 重置里程计
     */
    void reset();

    /**
     * @brief 获取处理统计信息
     * @param total_frames 总帧数
     * @param successful_frames 成功帧数
     * @param average_time 平均处理时间
     */
    void getStatistics(int& total_frames, int& successful_frames, double& average_time) const;

private:
    RegistrationMethod registration_method_;  // 配准方法
    std::unique_ptr<NDTRegistration> ndt_registration_;      // NDT配准器
    std::unique_ptr<GNICPRegistration> gn_icp_registration_; // GN-ICP配准器
    
    // 点云处理参数
    double voxel_size_;          // 体素大小
    double max_range_;           // 最大距离
    double min_range_;           // 最小距离
    
    // 里程计状态
    bool is_initialized_;        // 是否已初始化
    PointCloudPtr previous_cloud_;  // 前一帧点云
    Eigen::Matrix4d current_pose_;  // 当前位姿
    std::vector<TrajectoryPoint> trajectory_;  // 轨迹
    
    // 统计信息
    int total_frames_;           // 总帧数
    int successful_frames_;      // 成功帧数
    double total_processing_time_; // 总处理时间
    
    /**
     * @brief 预处理点云
     * @param cloud 输入点云
     * @return 预处理后的点云
     */
    PointCloudPtr preprocessPointCloud(const PointCloudPtr& cloud);

    /**
     * @brief 距离滤波
     * @param cloud 输入点云
     * @return 滤波后的点云
     */
    PointCloudPtr rangeFilter(const PointCloudPtr& cloud);

    /**
     * @brief 体素滤波
     * @param cloud 输入点云
     * @return 滤波后的点云
     */
    PointCloudPtr voxelFilter(const PointCloudPtr& cloud);

    /**
     * @brief 从变换矩阵提取位置和姿态
     * @param transformation 变换矩阵
     * @param position 输出位置
     * @param orientation 输出姿态四元数
     */
    void extractPose(const Eigen::Matrix4d& transformation,
                    Eigen::Vector3d& position,
                    Eigen::Quaterniond& orientation);

    /**
     * @brief 初始化配准器
     */
    void initializeRegistration();
};

}

#endif 
