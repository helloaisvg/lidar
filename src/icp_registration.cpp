#include "lidar_odometry/icp_registration.h"
#include <pcl/console/time.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <thread>
#include <chrono>

namespace lidar_odometry {

ICPRegistration::ICPRegistration(int max_iterations,
                                 double max_correspondence_distance,
                                 double transformation_epsilon,
                                 double euclidean_fitness_epsilon)
    : voxel_size_(0.0), has_converged_(false), fitness_score_(std::numeric_limits<double>::max()),
      final_transformation_(Eigen::Matrix4f::Identity()), num_iterations_(0) {
    
    // 创建PCL ICP对象
    icp_ = pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>::Ptr(
        new pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>);
    
    // 设置ICP参数
    icp_->setMaximumIterations(max_iterations);
    icp_->setMaxCorrespondenceDistance(max_correspondence_distance);
    icp_->setTransformationEpsilon(transformation_epsilon);
    icp_->setEuclideanFitnessEpsilon(euclidean_fitness_epsilon);
    icp_->setUseReciprocalCorrespondences(false);
}

Eigen::Matrix4f ICPRegistration::align(const PointCloudPtr& source,
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

    if (source_filtered->empty() || target_filtered->empty()) {
        std::cerr << "Error: Empty point clouds after filtering!" << std::endl;
        return Eigen::Matrix4f::Identity();
    }

    // 设置输入点云
    icp_->setInputSource(source_filtered);
    icp_->setInputTarget(target_filtered);

    // 执行配准
    PointCloudPtr aligned_cloud(new PointCloud);
    
    pcl::console::TicToc timer;
    timer.tic();
    
    icp_->align(*aligned_cloud, initial_guess);
    
    double alignment_time = timer.toc();

    // 获取结果
    has_converged_ = icp_->hasConverged();
    fitness_score_ = icp_->getFitnessScore();
    final_transformation_ = icp_->getFinalTransformation();
    // PCL ICP不直接提供迭代次数，使用最大迭代次数作为近似值
    num_iterations_ = icp_->getMaximumIterations();

    if (has_converged_) {
        std::cout << "Valid correspondence found: " << icp_->getFitnessScore() << std::endl;
    }

    std::cout << "ICP alignment took " << alignment_time << " ms" << std::endl;
    std::cout << "ICP Registration Results:" << std::endl;
    std::cout << "  Converged: " << (has_converged_ ? "Yes" : "No") << std::endl;
    std::cout << "  Fitness Score: " << fitness_score_ << std::endl;
    std::cout << "  Number of iterations: " << num_iterations_ << std::endl;

    return final_transformation_;
}

bool ICPRegistration::hasConverged() const {
    return has_converged_;
}

double ICPRegistration::getFitnessScore() const {
    return fitness_score_;
}

Eigen::Matrix4f ICPRegistration::getFinalTransformation() const {
    return final_transformation_;
}

int ICPRegistration::getNumIterations() const {
    return num_iterations_;
}

void ICPRegistration::setVoxelGridFilter(double voxel_size) {
    voxel_size_ = voxel_size;
}

ICPRegistration::PointCloudPtr ICPRegistration::preprocessPointCloud(const PointCloudPtr& cloud) {
    if (voxel_size_ <= 0.0) {
        // 不使用体素滤波，直接返回
        return cloud;
    }

    PointCloudPtr filtered_cloud(new PointCloud);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
    voxel_filter.filter(*filtered_cloud);

    return filtered_cloud;
}

void ICPRegistration::visualizeRegistration(const PointCloudPtr& source,
                                           const PointCloudPtr& target,
                                           const Eigen::Matrix4f& transformation) {
    // 变换源点云
    PointCloudPtr transformed_source(new PointCloud);
    pcl::transformPointCloud(*source, *transformed_source, transformation);
    
    // 创建可视化器
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("ICP Registration"));
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

void ICPRegistration::saveRegistrationScreenshot(const PointCloudPtr& source,
                                                const PointCloudPtr& target,
                                                const Eigen::Matrix4f& transformation,
                                                const std::string& filename) {
    // 变换源点云
    PointCloudPtr transformed_source(new PointCloud);
    pcl::transformPointCloud(*source, *transformed_source, transformation);
    
    // 创建可视化器
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("ICP Registration Screenshot"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->setShowFPS(false);
    viewer->setSize(1920, 1080);
    
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
    
    // 设置视角
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*target, min_pt, max_pt);
    double center_x = (min_pt.x + max_pt.x) / 2.0;
    double center_y = (min_pt.y + max_pt.y) / 2.0;
    double center_z = (min_pt.z + max_pt.z) / 2.0;
    double range = std::max({max_pt.x - min_pt.x, max_pt.y - min_pt.y, max_pt.z - min_pt.z});
    
    viewer->setCameraPosition(
        center_x + range * 0.5, center_y + range * 0.5, center_z + range * 0.5,
        center_x, center_y, center_z,
        0, 0, 1);
    
    // 等待渲染完成
    for (int i = 0; i < 10; ++i) {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // 保存截图
    viewer->saveScreenshot(filename);
    std::cout << "配准截图已保存: " << filename << std::endl;
    
    // 关闭可视化器
    viewer->close();
}

}  // namespace lidar_odometry

