#ifndef ICP_REGISTRATION_H
#define ICP_REGISTRATION_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include <string>

namespace lidar_odometry {

/**
 * @brief ICP配准类
 * 使用PCL库实现的标准ICP配准算法
 */
class ICPRegistration {
public:
    using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
    using PointCloudPtr = PointCloud::Ptr;

    /**
     * @brief 构造函数
     * @param max_iterations 最大迭代次数
     * @param max_correspondence_distance 最大对应距离
     * @param transformation_epsilon 变换收敛阈值
     * @param euclidean_fitness_epsilon 欧几里得适应度阈值
     */
    ICPRegistration(int max_iterations = 50,
                    double max_correspondence_distance = 1.0,
                    double transformation_epsilon = 1e-6,
                    double euclidean_fitness_epsilon = 1e-6);

    /**
     * @brief 析构函数
     */
    ~ICPRegistration() = default;

    /**
     * @brief 执行ICP配准
     * @param source 源点云
     * @param target 目标点云
     * @param initial_guess 初始变换猜测（默认单位矩阵）
     * @return 最终变换矩阵
     */
    Eigen::Matrix4f align(const PointCloudPtr& source,
                         const PointCloudPtr& target,
                         const Eigen::Matrix4f& initial_guess = Eigen::Matrix4f::Identity());

    /**
     * @brief 检查是否收敛
     * @return 是否收敛
     */
    bool hasConverged() const;

    /**
     * @brief 获取适应度分数
     * @return 适应度分数
     */
    double getFitnessScore() const;

    /**
     * @brief 获取最终变换矩阵
     * @return 最终变换矩阵
     */
    Eigen::Matrix4f getFinalTransformation() const;

    /**
     * @brief 获取迭代次数
     * @return 迭代次数
     */
    int getNumIterations() const;

    /**
     * @brief 设置体素滤波
     * @param voxel_size 体素大小
     */
    void setVoxelGridFilter(double voxel_size);

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
    /**
     * @brief 预处理点云（体素滤波）
     * @param cloud 输入点云
     * @return 处理后的点云
     */
    PointCloudPtr preprocessPointCloud(const PointCloudPtr& cloud);

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>::Ptr icp_;
    double voxel_size_;
    bool has_converged_;
    double fitness_score_;
    Eigen::Matrix4f final_transformation_;
    int num_iterations_;
};

}  // namespace lidar_odometry

#endif  // ICP_REGISTRATION_H

