#ifndef NDT_REGISTRATION_H
#define NDT_REGISTRATION_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Dense>
#include <iostream>
#include <memory>

namespace lidar_odometry {

/**
 * @brief NDT配准类
 * 基于PCL库实现NDT (Normal Distributions Transform) 配准算法
 */
class NDTRegistration {
public:
    using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
    using PointCloudPtr = PointCloud::Ptr;
    using NDT = pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>;
    using NDPtr = NDT::Ptr;

    /**
     * @brief 构造函数
     * @param resolution NDT网格分辨率
     * @param step_size 优化步长
     * @param max_iterations 最大迭代次数
     * @param transformation_epsilon 变换收敛阈值
     * @param euclidean_fitness_epsilon 欧几里得适应度阈值
     */
    NDTRegistration(double resolution = 1.0,
                   double step_size = 0.1,
                   int max_iterations = 35,
                   double transformation_epsilon = 0.01,
                   double euclidean_fitness_epsilon = 0.01);

    /**
     * @brief 析构函数
     */
    ~NDTRegistration() = default;

    /**
     * @brief 设置NDT参数
     * @param resolution 网格分辨率
     * @param step_size 优化步长
     * @param max_iterations 最大迭代次数
     * @param transformation_epsilon 变换收敛阈值
     * @param euclidean_fitness_epsilon 欧几里得适应度阈值
     */
    void setParameters(double resolution,
                      double step_size,
                      int max_iterations,
                      double transformation_epsilon,
                      double euclidean_fitness_epsilon);

    /**
     * @brief 执行NDT配准
     * @param source 源点云
     * @param target 目标点云
     * @param initial_guess 初始变换矩阵
     * @return 配准后的变换矩阵
     */
    Eigen::Matrix4f align(const PointCloudPtr& source,
                         const PointCloudPtr& target,
                         const Eigen::Matrix4f& initial_guess = Eigen::Matrix4f::Identity());

    /**
     * @brief 获取配准分数
     * @return 配准分数
     */
    double getFitnessScore() const;

    /**
     * @brief 获取最终变换矩阵
     * @return 最终变换矩阵
     */
    Eigen::Matrix4f getFinalTransformation() const;

    /**
     * @brief 检查配准是否收敛
     * @return 是否收敛
     */
    bool hasConverged() const;

    /**
     * @brief 设置体素网格下采样
     * @param leaf_size 体素大小
     */
    void setVoxelGridFilter(double leaf_size);

    /**
     * @brief 预处理点云
     * @param cloud 输入点云
     * @return 预处理后的点云
     */
    PointCloudPtr preprocessPointCloud(const PointCloudPtr& cloud);

    /**
     * @brief 可视化配准结果
     * @param source 源点云
     * @param target 目标点云
     * @param transformation 变换矩阵
     */
    void visualizeRegistration(const PointCloudPtr& source,
                              const PointCloudPtr& target,
                              const Eigen::Matrix4f& transformation);

    /**
     * @brief 保存配准结果截图
     * @param source 源点云
     * @param target 目标点云
     * @param transformation 变换矩阵
     * @param filename 保存的文件名
     */
    void saveRegistrationScreenshot(const PointCloudPtr& source,
                                   const PointCloudPtr& target,
                                   const Eigen::Matrix4f& transformation,
                                   const std::string& filename);

private:
    NDPtr ndt_;                    // NDT配准器
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_;  // 体素网格滤波器
    double leaf_size_;             // 体素大小
    bool use_voxel_filter_;        // 是否使用体素滤波
    
    // 配准结果
    double fitness_score_;         // 配准分数
    Eigen::Matrix4f final_transformation_;  // 最终变换矩阵
    bool has_converged_;           // 是否收敛

    /**
     * @brief 初始化NDT参数
     */
    void initializeNDT();
};

} 

#endif 
