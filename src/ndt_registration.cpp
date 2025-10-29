#include "lidar_odometry/ndt_registration.h"
#include <pcl/console/time.h>
#include <pcl/visualization/cloud_viewer.h>
#include <chrono>
#include <thread>

namespace lidar_odometry {

NDTRegistration::NDTRegistration(double resolution,
                                 double step_size,
                                 int max_iterations,
                                 double transformation_epsilon,
                                 double euclidean_fitness_epsilon)
    : leaf_size_(0.1)
    , use_voxel_filter_(true)
    , fitness_score_(std::numeric_limits<double>::max())
    , final_transformation_(Eigen::Matrix4f::Identity())
    , has_converged_(false) {
    
    // 创建NDT配准器
    ndt_ = NDPtr(new NDT());
    
    // 设置参数
    setParameters(resolution, step_size, max_iterations, 
                 transformation_epsilon, euclidean_fitness_epsilon);
    
    // 初始化体素滤波器
    voxel_filter_.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
}

void NDTRegistration::setParameters(double resolution,
                                   double step_size,
                                   int max_iterations,
                                   double transformation_epsilon,
                                   double euclidean_fitness_epsilon) {
    if (ndt_) {
        ndt_->setResolution(resolution);
        ndt_->setStepSize(step_size);
        ndt_->setMaximumIterations(max_iterations);
        ndt_->setTransformationEpsilon(transformation_epsilon);
        ndt_->setEuclideanFitnessEpsilon(euclidean_fitness_epsilon);
    }
}

Eigen::Matrix4f NDTRegistration::align(const PointCloudPtr& source,
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

    // 设置输入点云
    ndt_->setInputSource(source_filtered);
    ndt_->setInputTarget(target_filtered);

    // 执行配准
    PointCloudPtr aligned_cloud(new PointCloud);
    pcl::console::TicToc timer;
    timer.tic();
    
    ndt_->align(*aligned_cloud, initial_guess);
    
    double alignment_time = timer.toc();
    std::cout << "NDT alignment took " << alignment_time << " ms" << std::endl;

    // 获取结果
    final_transformation_ = ndt_->getFinalTransformation();
    fitness_score_ = ndt_->getFitnessScore();
    has_converged_ = ndt_->hasConverged();

    std::cout << "NDT Registration Results:" << std::endl;
    std::cout << "  Converged: " << (has_converged_ ? "Yes" : "No") << std::endl;
    std::cout << "  Fitness Score: " << fitness_score_ << std::endl;
    std::cout << "  Number of iterations: " << ndt_->getFinalNumIteration() << std::endl;

    return final_transformation_;
}

double NDTRegistration::getFitnessScore() const {
    return fitness_score_;
}

Eigen::Matrix4f NDTRegistration::getFinalTransformation() const {
    return final_transformation_;
}

bool NDTRegistration::hasConverged() const {
    return has_converged_;
}

void NDTRegistration::setVoxelGridFilter(double leaf_size) {
    leaf_size_ = leaf_size;
    voxel_filter_.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
    use_voxel_filter_ = true;
}

PointCloudPtr NDTRegistration::preprocessPointCloud(const PointCloudPtr& cloud) {
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

void NDTRegistration::visualizeRegistration(const PointCloudPtr& source,
                                           const PointCloudPtr& target,
                                           const Eigen::Matrix4f& transformation) {
    // 变换源点云
    PointCloudPtr transformed_source(new PointCloud);
    pcl::transformPointCloud(*source, *transformed_source, transformation);
    
    // 创建可视化器
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("NDT Registration"));
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

void NDTRegistration::initializeNDT() {
    if (!ndt_) {
        ndt_ = NDPtr(new NDT());
    }
}

} 