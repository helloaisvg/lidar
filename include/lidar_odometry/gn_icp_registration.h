#ifndef GN_ICP_REGISTRATION_H
#define GN_ICP_REGISTRATION_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <memory>
#include <vector>
#include <algorithm>

namespace lidar_odometry {

/**
 * @brief GN-ICP配准类
 * 手写实现Gauss-Newton ICP配准算法
 */
class GNICPRegistration {
public:
    using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
    using PointCloudPtr = PointCloud::Ptr;
    using KdTree = pcl::KdTreeFLANN<pcl::PointXYZ>;
    using KdTreePtr = KdTree::Ptr;

    /**
     * @brief 构造函数
     * @param max_iterations 最大迭代次数
     * @param max_correspondence_distance 最大对应距离
     * @param transformation_epsilon 变换收敛阈值
     * @param euclidean_fitness_epsilon 欧几里得适应度阈值
     * @param use_reciprocal_correspondences 是否使用互相对应
     * @param use_linear_interpolation 是否使用线性插值
     */
    GNICPRegistration(int max_iterations = 50,
                     double max_correspondence_distance = 0.5,
                     double transformation_epsilon = 1e-6,
                     double euclidean_fitness_epsilon = 1e-6,
                     bool use_reciprocal_correspondences = true,
                     bool use_linear_interpolation = true);

    /**
     * @brief 析构函数
     */
    ~GNICPRegistration() = default;

    /**
     * @brief 设置配准参数
     * @param max_iterations 最大迭代次数
     * @param max_correspondence_distance 最大对应距离
     * @param transformation_epsilon 变换收敛阈值
     * @param euclidean_fitness_epsilon 欧几里得适应度阈值
     * @param use_reciprocal_correspondences 是否使用互相对应
     * @param use_linear_interpolation 是否使用线性插值
     */
    void setParameters(int max_iterations,
                      double max_correspondence_distance,
                      double transformation_epsilon,
                      double euclidean_fitness_epsilon,
                      bool use_reciprocal_correspondences = true,
                      bool use_linear_interpolation = true);

    /**
     * @brief 执行GN-ICP配准
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
     * @brief 获取迭代次数
     * @return 迭代次数
     */
    int getFinalNumIteration() const;

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

private:
    // 配准参数
    int max_iterations_;                    // 最大迭代次数
    double max_correspondence_distance_;     // 最大对应距离
    double transformation_epsilon_;          // 变换收敛阈值
    double euclidean_fitness_epsilon_;       // 欧几里得适应度阈值
    bool use_reciprocal_correspondences_;    // 是否使用互相对应
    bool use_linear_interpolation_;         // 是否使用线性插值
    
    // 体素滤波参数
    double leaf_size_;                       // 体素大小
    bool use_voxel_filter_;                  // 是否使用体素滤波
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_;  // 体素网格滤波器
    
    // 配准结果
    double fitness_score_;                   // 配准分数
    Eigen::Matrix4f final_transformation_;  // 最终变换矩阵
    bool has_converged_;                     // 是否收敛
    int final_num_iteration_;                // 最终迭代次数

    /**
     * @brief 计算点云之间的对应关系
     * @param source 源点云
     * @param target 目标点云
     * @param correspondences 输出对应关系
     * @param distances 输出距离
     */
    void computeCorrespondences(const PointCloudPtr& source,
                               const PointCloudPtr& target,
                               std::vector<int>& correspondences,
                               std::vector<float>& distances);

    /**
     * @brief 计算变换矩阵的雅可比矩阵
     * @param point 点
     * @param jacobian 输出雅可比矩阵
     */
    void computeJacobian(const pcl::PointXYZ& point, Eigen::Matrix<double, 3, 6>& jacobian);

    /**
     * @brief 从变换矩阵提取旋转和平移
     * @param transformation 变换矩阵
     * @param rotation 输出旋转矩阵
     * @param translation 输出平移向量
     */
    void extractRotationTranslation(const Eigen::Matrix4f& transformation,
                                   Eigen::Matrix3f& rotation,
                                   Eigen::Vector3f& translation);

    /**
     * @brief 构建变换矩阵
     * @param rotation 旋转矩阵
     * @param translation 平移向量
     * @return 变换矩阵
     */
    Eigen::Matrix4f buildTransformationMatrix(const Eigen::Matrix3f& rotation,
                                             const Eigen::Vector3f& translation);

    /**
     * @brief 计算点云质心
     * @param cloud 点云
     * @return 质心
     */
    Eigen::Vector3f computeCentroid(const PointCloudPtr& cloud);

    /**
     * @brief 计算点云协方差矩阵
     * @param cloud 点云
     * @param centroid 质心
     * @return 协方差矩阵
     */
    Eigen::Matrix3f computeCovarianceMatrix(const PointCloudPtr& cloud,
                                           const Eigen::Vector3f& centroid);
};

} 

#endif 
