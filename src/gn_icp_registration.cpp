#include "lidar_odometry/gn_icp_registration.h"
#include <pcl/console/time.h>
#include <pcl/visualization/cloud_viewer.h>
#include <chrono>
#include <thread>
#include <cmath>

namespace lidar_odometry {

GNICPRegistration::GNICPRegistration(int max_iterations,
                                   double max_correspondence_distance,
                                   double transformation_epsilon,
                                   double euclidean_fitness_epsilon,
                                   bool use_reciprocal_correspondences,
                                   bool use_linear_interpolation)
    : max_iterations_(max_iterations)
    , max_correspondence_distance_(max_correspondence_distance)
    , transformation_epsilon_(transformation_epsilon)
    , euclidean_fitness_epsilon_(euclidean_fitness_epsilon)
    , use_reciprocal_correspondences_(use_reciprocal_correspondences)
    , use_linear_interpolation_(use_linear_interpolation)
    , leaf_size_(0.1)
    , use_voxel_filter_(true)
    , fitness_score_(std::numeric_limits<double>::max())
    , final_transformation_(Eigen::Matrix4f::Identity())
    , has_converged_(false)
    , final_num_iteration_(0) {
    
    // 初始化体素滤波器
    voxel_filter_.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
}

void GNICPRegistration::setParameters(int max_iterations,
                                     double max_correspondence_distance,
                                     double transformation_epsilon,
                                     double euclidean_fitness_epsilon,
                                     bool use_reciprocal_correspondences,
                                     bool use_linear_interpolation) {
    max_iterations_ = max_iterations;
    max_correspondence_distance_ = max_correspondence_distance;
    transformation_epsilon_ = transformation_epsilon;
    euclidean_fitness_epsilon_ = euclidean_fitness_epsilon;
    use_reciprocal_correspondences_ = use_reciprocal_correspondences;
    use_linear_interpolation_ = use_linear_interpolation;
}

Eigen::Matrix4f GNICPRegistration::align(const PointCloudPtr& source,
                                         const PointCloudPtr& target,
                                         const Eigen::Matrix4f& initial_guess) {
    if (!source || !target) {
        std::cerr << "Error: Invalid point clouds!" << std::endl;
        return Eigen::Matrix4f::Identity();
    }

    if (source->empty() || target->empty()) {
        std::cerr << "Error: Empty point clouds!" << std::endl;
        return Eigen::Matrix4f::Identity();
    }

    // 预处理点云
    PointCloudPtr source_filtered = preprocessPointCloud(source);
    PointCloudPtr target_filtered = preprocessPointCloud(target);

    // 初始化变换矩阵
    Eigen::Matrix4f transformation = initial_guess;
    final_transformation_ = transformation;

    // 创建KD树用于最近邻搜索
    KdTreePtr kdtree(new KdTree);
    kdtree->setInputCloud(target_filtered);

    // 迭代优化
    double previous_fitness_score = std::numeric_limits<double>::max();
    fitness_score_ = std::numeric_limits<double>::max();
    has_converged_ = false;

    pcl::console::TicToc timer;
    timer.tic();

    for (int iteration = 0; iteration < max_iterations_; ++iteration) {
        // 变换源点云
        PointCloudPtr transformed_source(new PointCloud);
        pcl::transformPointCloud(*source_filtered, *transformed_source, transformation);

        // 计算对应关系
        std::vector<int> correspondences;
        std::vector<float> distances;
        computeCorrespondences(transformed_source, target_filtered, correspondences, distances);

        if (correspondences.empty()) {
            std::cout << "No correspondences found!" << std::endl;
            break;
        }

        // 计算当前适应度分数
        double current_fitness_score = 0.0;
        for (size_t i = 0; i < correspondences.size(); ++i) {
            if (correspondences[i] >= 0) {
                current_fitness_score += distances[i];
            }
        }
        current_fitness_score /= correspondences.size();

        // 检查收敛
        if (std::abs(current_fitness_score - previous_fitness_score) < euclidean_fitness_epsilon_) {
            has_converged_ = true;
            fitness_score_ = current_fitness_score;
            final_num_iteration_ = iteration + 1;
            break;
        }

        previous_fitness_score = current_fitness_score;

        // 构建线性系统 Ax = b
        Eigen::Matrix<double, 6, 6> A = Eigen::Matrix<double, 6, 6>::Zero();
        Eigen::Matrix<double, 6, 1> b = Eigen::Matrix<double, 6, 1>::Zero();

        for (size_t i = 0; i < correspondences.size(); ++i) {
            if (correspondences[i] >= 0) {
                const pcl::PointXYZ& source_point = transformed_source->points[i];
                const pcl::PointXYZ& target_point = target_filtered->points[correspondences[i]];

                // 计算误差向量
                Eigen::Vector3d error;
                error << target_point.x - source_point.x,
                        target_point.y - source_point.y,
                        target_point.z - source_point.z;

                // 计算雅可比矩阵
                Eigen::Matrix<double, 3, 6> jacobian;
                computeJacobian(source_point, jacobian);

                // 累积到线性系统
                A += jacobian.transpose() * jacobian;
                b += jacobian.transpose() * error;
            }
        }

        // 求解线性系统
        Eigen::Matrix<double, 6, 1> delta = A.ldlt().solve(b);

        // 检查变换收敛
        if (delta.norm() < transformation_epsilon_) {
            has_converged_ = true;
            fitness_score_ = current_fitness_score;
            final_num_iteration_ = iteration + 1;
            break;
        }

        // 更新变换矩阵
        Eigen::Matrix3f delta_rotation = Eigen::Matrix3f::Identity();
        delta_rotation(0, 1) = -delta(2);
        delta_rotation(0, 2) = delta(1);
        delta_rotation(1, 0) = delta(2);
        delta_rotation(1, 2) = -delta(0);
        delta_rotation(2, 0) = -delta(1);
        delta_rotation(2, 1) = delta(0);

        Eigen::Vector3f delta_translation;
        delta_translation << delta(3), delta(4), delta(5);

        Eigen::Matrix4f delta_transformation = Eigen::Matrix4f::Identity();
        delta_transformation.block<3, 3>(0, 0) = delta_rotation;
        delta_transformation.block<3, 1>(0, 3) = delta_translation;

        transformation = transformation * delta_transformation;
        final_transformation_ = transformation;
    }

    double alignment_time = timer.toc();
    std::cout << "GN-ICP alignment took " << alignment_time << " ms" << std::endl;

    std::cout << "GN-ICP Registration Results:" << std::endl;
    std::cout << "  Converged: " << (has_converged_ ? "Yes" : "No") << std::endl;
    std::cout << "  Fitness Score: " << fitness_score_ << std::endl;
    std::cout << "  Number of iterations: " << final_num_iteration_ << std::endl;

    return final_transformation_;
}

void GNICPRegistration::computeCorrespondences(const PointCloudPtr& source,
                                             const PointCloudPtr& target,
                                             std::vector<int>& correspondences,
                                             std::vector<float>& distances) {
    correspondences.clear();
    distances.clear();
    correspondences.resize(source->size(), -1);
    distances.resize(source->size(), std::numeric_limits<float>::max());

    KdTreePtr kdtree(new KdTree);
    kdtree->setInputCloud(target);

    for (size_t i = 0; i < source->size(); ++i) {
        const pcl::PointXYZ& query_point = source->points[i];
        
        std::vector<int> indices(1);
        std::vector<float> sqr_distances(1);
        
        if (kdtree->nearestKSearch(query_point, 1, indices, sqr_distances) > 0) {
            float distance = std::sqrt(sqr_distances[0]);
            if (distance < max_correspondence_distance_) {
                correspondences[i] = indices[0];
                distances[i] = distance;
            }
        }
    }
}

void GNICPRegistration::computeJacobian(const pcl::PointXYZ& point, 
                                       Eigen::Matrix<double, 3, 6>& jacobian) {
    jacobian.setZero();
    
    // 旋转部分的雅可比矩阵
    jacobian(0, 1) = point.z;
    jacobian(0, 2) = -point.y;
    jacobian(1, 0) = -point.z;
    jacobian(1, 2) = point.x;
    jacobian(2, 0) = point.y;
    jacobian(2, 1) = -point.x;
    
    // 平移部分的雅可比矩阵
    jacobian(0, 3) = 1.0;
    jacobian(1, 4) = 1.0;
    jacobian(2, 5) = 1.0;
}

void GNICPRegistration::extractRotationTranslation(const Eigen::Matrix4f& transformation,
                                                 Eigen::Matrix3f& rotation,
                                                 Eigen::Vector3f& translation) {
    rotation = transformation.block<3, 3>(0, 0);
    translation = transformation.block<3, 1>(0, 3);
}

Eigen::Matrix4f GNICPRegistration::buildTransformationMatrix(const Eigen::Matrix3f& rotation,
                                                            const Eigen::Vector3f& translation) {
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
    transformation.block<3, 3>(0, 0) = rotation;
    transformation.block<3, 1>(0, 3) = translation;
    return transformation;
}

Eigen::Vector3f GNICPRegistration::computeCentroid(const PointCloudPtr& cloud) {
    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    for (const auto& point : cloud->points) {
        centroid += Eigen::Vector3f(point.x, point.y, point.z);
    }
    centroid /= cloud->size();
    return centroid;
}

Eigen::Matrix3f GNICPRegistration::computeCovarianceMatrix(const PointCloudPtr& cloud,
                                                         const Eigen::Vector3f& centroid) {
    Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
    for (const auto& point : cloud->points) {
        Eigen::Vector3f diff = Eigen::Vector3f(point.x, point.y, point.z) - centroid;
        covariance += diff * diff.transpose();
    }
    covariance /= cloud->size();
    return covariance;
}

double GNICPRegistration::getFitnessScore() const {
    return fitness_score_;
}

Eigen::Matrix4f GNICPRegistration::getFinalTransformation() const {
    return final_transformation_;
}

bool GNICPRegistration::hasConverged() const {
    return has_converged_;
}

int GNICPRegistration::getFinalNumIteration() const {
    return final_num_iteration_;
}

void GNICPRegistration::setVoxelGridFilter(double leaf_size) {
    leaf_size_ = leaf_size;
    voxel_filter_.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
    use_voxel_filter_ = true;
}

PointCloudPtr GNICPRegistration::preprocessPointCloud(const PointCloudPtr& cloud) {
    PointCloudPtr filtered_cloud(new PointCloud);
    
    if (use_voxel_filter_) {
        voxel_filter_.setInputCloud(cloud);
        voxel_filter_.filter(*filtered_cloud);
        
        std::cout << "Original cloud size: " << cloud->size() 
                  << ", Filtered cloud size: " << filtered_cloud->size() << std::endl;
    } else {
        *filtered_cloud = *cloud;
    }
    
    return filtered_cloud;
}

void GNICPRegistration::visualizeRegistration(const PointCloudPtr& source,
                                            const PointCloudPtr& target,
                                            const Eigen::Matrix4f& transformation) {
    // 变换源点云
    PointCloudPtr transformed_source(new PointCloud);
    pcl::transformPointCloud(*source, *transformed_source, transformation);
    
    // 创建可视化器
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("GN-ICP Registration"));
    viewer->setBackgroundColor(0, 0, 0);
    
    // 添加目标点云（绿色）
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(target, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZ>(target, target_color, "target_cloud");
    
    // 添加变换后的源点云（红色）
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color(transformed_source, 255, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(transformed_source, source_color, "source_cloud");
    
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target_cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source_cloud");
    
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    
    std::cout << "Press 'q' to quit the visualization" << std::endl;
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

} 