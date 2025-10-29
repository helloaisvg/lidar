#include "lidar_odometry/lidar_odometry.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <filesystem>
#include <sstream>
#include <iomanip>

namespace lidar_odometry {

LidarOdometry::LidarOdometry(RegistrationMethod method,
                             double voxel_size,
                             double max_range,
                             double min_range)
    : registration_method_(method)
    , voxel_size_(voxel_size)
    , max_range_(max_range)
    , min_range_(min_range)
    , is_initialized_(false)
    , current_pose_(Eigen::Matrix4d::Identity())
    , total_frames_(0)
    , successful_frames_(0)
    , total_processing_time_(0.0) {
    
    initializeRegistration();
}

void LidarOdometry::setRegistrationMethod(RegistrationMethod method) {
    registration_method_ = method;
    initializeRegistration();
}

void LidarOdometry::setVoxelSize(double voxel_size) {
    voxel_size_ = voxel_size;
    if (ndt_registration_) {
        ndt_registration_->setVoxelGridFilter(voxel_size_);
    }
    if (gn_icp_registration_) {
        gn_icp_registration_->setVoxelGridFilter(voxel_size_);
    }
}

void LidarOdometry::setRangeFilter(double min_range, double max_range) {
    min_range_ = min_range;
    max_range_ = max_range;
}

bool LidarOdometry::processFrame(const PointCloudPtr& cloud, double timestamp) {
    if (!cloud || cloud->empty()) {
        std::cerr << "Error: Invalid or empty point cloud!" << std::endl;
        return false;
    }

    total_frames_++;
    pcl::console::TicToc timer;
    timer.tic();

    // 预处理点云
    PointCloudPtr processed_cloud = preprocessPointCloud(cloud);
    if (!processed_cloud || processed_cloud->empty()) {
        std::cerr << "Error: Point cloud preprocessing failed!" << std::endl;
        return false;
    }

    if (!is_initialized_) {
        // 第一帧，初始化
        previous_cloud_ = processed_cloud;
        current_pose_ = Eigen::Matrix4d::Identity();
        
        // 添加轨迹点
        Eigen::Vector3d position = Eigen::Vector3d::Zero();
        Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
        trajectory_.emplace_back(timestamp, position, orientation);
        
        is_initialized_ = true;
        successful_frames_++;
        total_processing_time_ += timer.toc();
        
        std::cout << "Initialized with first frame" << std::endl;
        return true;
    }

    // 执行配准
    Eigen::Matrix4f relative_transform = Eigen::Matrix4f::Identity();
    bool registration_success = false;

    if (registration_method_ == RegistrationMethod::NDT && ndt_registration_) {
        relative_transform = ndt_registration_->align(processed_cloud, previous_cloud_);
        registration_success = ndt_registration_->hasConverged();
    } else if (registration_method_ == RegistrationMethod::GN_ICP && gn_icp_registration_) {
        relative_transform = gn_icp_registration_->align(processed_cloud, previous_cloud_);
        registration_success = gn_icp_registration_->hasConverged();
    }

    if (!registration_success) {
        std::cerr << "Registration failed!" << std::endl;
        return false;
    }

    // 更新位姿
    Eigen::Matrix4d relative_transform_d = relative_transform.cast<double>();
    current_pose_ = current_pose_ * relative_transform_d;

    // 提取位置和姿态
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    extractPose(current_pose_, position, orientation);

    // 添加轨迹点
    trajectory_.emplace_back(timestamp, position, orientation);

    // 更新前一帧点云
    previous_cloud_ = processed_cloud;

    successful_frames_++;
    total_processing_time_ += timer.toc();

    std::cout << "Frame " << total_frames_ 
              << " processed successfully. Position: [" 
              << position.transpose() << "]" << std::endl;

    return true;
}

bool LidarOdometry::processKITTISequence(const std::string& data_path, int sequence) {
    std::string sequence_path = data_path + "/sequences/" + std::to_string(sequence).substr(0, 2) + "/velodyne/";
    
    if (!std::filesystem::exists(sequence_path)) {
        std::cerr << "Error: KITTI sequence path does not exist: " << sequence_path << std::endl;
        return false;
    }

    std::cout << "Processing KITTI sequence " << sequence << " from: " << sequence_path << std::endl;

    // 获取所有点云文件
    std::vector<std::string> cloud_files;
    for (const auto& entry : std::filesystem::directory_iterator(sequence_path)) {
        if (entry.path().extension() == ".bin") {
            cloud_files.push_back(entry.path().string());
        }
    }
    
    std::sort(cloud_files.begin(), cloud_files.end());

    std::cout << "Found " << cloud_files.size() << " point cloud files" << std::endl;

    // 处理每一帧
    for (size_t i = 0; i < cloud_files.size(); ++i) {
        // 读取点云
        PointCloudPtr cloud(new PointCloud);
        if (pcl::io::loadPCDFile(cloud_files[i], *cloud) == -1) {
            // 尝试读取KITTI二进制格式
            std::ifstream file(cloud_files[i], std::ios::binary);
            if (!file.is_open()) {
                std::cerr << "Error: Cannot open file " << cloud_files[i] << std::endl;
                continue;
            }

            cloud->clear();
            float x, y, z, intensity;
            while (file.read(reinterpret_cast<char*>(&x), sizeof(float)) &&
                   file.read(reinterpret_cast<char*>(&y), sizeof(float)) &&
                   file.read(reinterpret_cast<char*>(&z), sizeof(float)) &&
                   file.read(reinterpret_cast<char*>(&intensity), sizeof(float))) {
                pcl::PointXYZ point;
                point.x = x;
                point.y = y;
                point.z = z;
                cloud->push_back(point);
            }
            file.close();
        }

        if (cloud->empty()) {
            std::cerr << "Error: Empty point cloud at frame " << i << std::endl;
            continue;
        }

        // 处理帧
        double timestamp = static_cast<double>(i) * 0.1; // 假设10Hz
        if (!processFrame(cloud, timestamp)) {
            std::cerr << "Error: Failed to process frame " << i << std::endl;
            continue;
        }

        // 每100帧输出一次进度
        if (i % 100 == 0) {
            std::cout << "Processed " << i << "/" << cloud_files.size() << " frames" << std::endl;
        }
    }

    std::cout << "KITTI sequence processing completed!" << std::endl;
    std::cout << "Total frames: " << total_frames_ << std::endl;
    std::cout << "Successful frames: " << successful_frames_ << std::endl;
    std::cout << "Success rate: " << (double)successful_frames_ / total_frames_ * 100.0 << "%" << std::endl;

    return true;
}

Eigen::Matrix4d LidarOdometry::getCurrentPose() const {
    return current_pose_;
}

const std::vector<LidarOdometry::TrajectoryPoint>& LidarOdometry::getTrajectory() const {
    return trajectory_;
}

bool LidarOdometry::saveTrajectory(const std::string& filename) const {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Cannot open file for writing: " << filename << std::endl;
        return false;
    }

    // 保存为TUM格式
    file << "# timestamp tx ty tz qx qy qz qw" << std::endl;
    for (const auto& point : trajectory_) {
        file << std::fixed << std::setprecision(6) 
             << point.timestamp << " "
             << point.position.x() << " " << point.position.y() << " " << point.position.z() << " "
             << point.orientation.x() << " " << point.orientation.y() << " " 
             << point.orientation.z() << " " << point.orientation.w() << std::endl;
    }

    file.close();
    std::cout << "Trajectory saved to: " << filename << std::endl;
    return true;
}

bool LidarOdometry::loadTrajectory(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Cannot open file for reading: " << filename << std::endl;
        return false;
    }

    trajectory_.clear();
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;

        std::istringstream iss(line);
        double timestamp, tx, ty, tz, qx, qy, qz, qw;
        if (iss >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw) {
            Eigen::Vector3d position(tx, ty, tz);
            Eigen::Quaterniond orientation(qw, qx, qy, qz);
            trajectory_.emplace_back(timestamp, position, orientation);
        }
    }

    file.close();
    std::cout << "Trajectory loaded from: " << filename << " (" << trajectory_.size() << " points)" << std::endl;
    return true;
}

void LidarOdometry::visualizeTrajectory(bool show_clouds) {
    if (trajectory_.empty()) {
        std::cout << "No trajectory to visualize!" << std::endl;
        return;
    }

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Trajectory Visualization"));
    viewer->setBackgroundColor(0, 0, 0);

    // 绘制轨迹
    pcl::PointCloud<pcl::PointXYZ> trajectory_cloud;
    for (const auto& point : trajectory_) {
        pcl::PointXYZ p;
        p.x = point.position.x();
        p.y = point.position.y();
        p.z = point.position.z();
        trajectory_cloud.push_back(p);
    }

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> trajectory_color(&trajectory_cloud, 255, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(trajectory_cloud.makeShared(), trajectory_color, "trajectory");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "trajectory");

    // 添加坐标系
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    std::cout << "Press 'q' to quit the visualization" << std::endl;
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void LidarOdometry::reset() {
    is_initialized_ = false;
    previous_cloud_.reset();
    current_pose_ = Eigen::Matrix4d::Identity();
    trajectory_.clear();
    total_frames_ = 0;
    successful_frames_ = 0;
    total_processing_time_ = 0.0;
}

void LidarOdometry::getStatistics(int& total_frames, int& successful_frames, double& average_time) const {
    total_frames = total_frames_;
    successful_frames = successful_frames_;
    average_time = total_frames_ > 0 ? total_processing_time_ / total_frames_ : 0.0;
}

PointCloudPtr LidarOdometry::preprocessPointCloud(const PointCloudPtr& cloud) {
    // 距离滤波
    PointCloudPtr range_filtered = rangeFilter(cloud);
    
    // 体素滤波
    PointCloudPtr voxel_filtered = voxelFilter(range_filtered);
    
    return voxel_filtered;
}

PointCloudPtr LidarOdometry::rangeFilter(const PointCloudPtr& cloud) {
    PointCloudPtr filtered_cloud(new PointCloud);
    
    for (const auto& point : cloud->points) {
        double distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
        if (distance >= min_range_ && distance <= max_range_) {
            filtered_cloud->push_back(point);
        }
    }
    
    return filtered_cloud;
}

PointCloudPtr LidarOdometry::voxelFilter(const PointCloudPtr& cloud) {
    PointCloudPtr filtered_cloud(new PointCloud);
    
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
    voxel_filter.filter(*filtered_cloud);
    
    return filtered_cloud;
}

void LidarOdometry::extractPose(const Eigen::Matrix4d& transformation,
                               Eigen::Vector3d& position,
                               Eigen::Quaterniond& orientation) {
    position = transformation.block<3, 1>(0, 3);
    Eigen::Matrix3d rotation_matrix = transformation.block<3, 3>(0, 0);
    orientation = Eigen::Quaterniond(rotation_matrix);
}

void LidarOdometry::initializeRegistration() {
    if (registration_method_ == RegistrationMethod::NDT) {
        ndt_registration_ = std::make_unique<NDTRegistration>(1.0, 0.1, 35, 0.01, 0.01);
        ndt_registration_->setVoxelGridFilter(voxel_size_);
    } else if (registration_method_ == RegistrationMethod::GN_ICP) {
        gn_icp_registration_ = std::make_unique<GNICPRegistration>(50, 0.5, 1e-6, 1e-6, true, true);
        gn_icp_registration_->setVoxelGridFilter(voxel_size_);
    }
}

} 