#include "lidar_odometry/ros_odometry_node.h"
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace lidar_odometry {

ROSOdometryNode::ROSOdometryNode(ros::NodeHandle& nh)
    : nh_(nh)
    , private_nh_("~")
    , is_initialized_(false)
    , is_running_(false)
    , last_cloud_time_(0.0)
    , last_imu_time_(0.0) {
}

bool ROSOdometryNode::initialize() {
    // 加载参数
    if (!loadParameters()) {
        ROS_ERROR("Failed to load parameters!");
        return false;
    }

    // 创建里程计
    odometry_ = std::make_unique<LidarOdometry>(registration_method_, voxel_size_, max_range_, min_range_);
    
    // 创建IMU积分器
    imu_integration_ = std::make_unique<IMUIntegration>();

    // 初始化路径
    path_.header.frame_id = odom_frame_;

    // 创建订阅者
    pointcloud_sub_ = nh_.subscribe(pointcloud_topic_, 1, &ROSOdometryNode::pointCloudCallback, this);
    imu_sub_ = nh_.subscribe(imu_topic_, 100, &ROSOdometryNode::imuCallback, this);

    // 创建发布者
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(pose_topic_, 1);
    odometry_pub_ = nh_.advertise<nav_msgs::Odometry>(odometry_topic_, 1);
    path_pub_ = nh_.advertise<nav_msgs::Path>(path_topic_, 1);
    
    if (publish_transformed_cloud_) {
        transformed_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(transformed_cloud_topic_, 1);
    }

    is_initialized_ = true;
    ROS_INFO("ROS Odometry Node initialized successfully!");
    return true;
}

void ROSOdometryNode::run() {
    if (!is_initialized_) {
        ROS_ERROR("Node not initialized!");
        return;
    }

    is_running_ = true;
    ROS_INFO("ROS Odometry Node started!");
    
    ros::spin();
}

void ROSOdometryNode::stop() {
    is_running_ = false;
    ROS_INFO("ROS Odometry Node stopped!");
}

void ROSOdometryNode::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
    if (!is_running_) return;

    // 转换点云格式
    PointCloudPtr cloud(new PointCloud);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    if (cloud->empty()) {
        ROS_WARN("Received empty point cloud!");
        return;
    }

    // 获取时间戳
    double timestamp = cloud_msg->header.stamp.toSec();
    last_cloud_time_ = timestamp;

    // 使用IMU预测（如果可用）
    Eigen::Matrix4d initial_guess = Eigen::Matrix4d::Identity();
    if (use_imu_prediction_ && imu_integration_->getIMUDataCount() > 0) {
        // 获取IMU预测的位姿变换
        double prev_time = last_cloud_time_ - 0.1; // 假设10Hz
        initial_guess = imu_integration_->predictPoseTransformation(prev_time, timestamp);
    }

    // 处理点云
    bool success = odometry_->processFrame(cloud, timestamp);
    
    if (success) {
        // 获取当前位姿
        Eigen::Matrix4d current_pose = odometry_->getCurrentPose();
        
        // 发布位姿
        publishPose(current_pose, cloud_msg->header.stamp);
        
        // 发布里程计
        publishOdometry(current_pose, cloud_msg->header.stamp);
        
        // 发布路径
        publishPath(current_pose, cloud_msg->header.stamp);
        
        // 发布变换后点云
        if (publish_transformed_cloud_) {
            publishTransformedCloud(cloud, current_pose, cloud_msg->header.stamp);
        }
        
        // 发布TF变换
        publishTransform(current_pose, cloud_msg->header.stamp);
    } else {
        ROS_WARN("Failed to process point cloud frame!");
    }
}

void ROSOdometryNode::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
    if (!is_running_) return;

    // 转换IMU数据
    IMUData imu_data;
    imu_data.timestamp = imu_msg->header.stamp.toSec();
    imu_data.linear_acceleration << imu_msg->linear_acceleration.x,
                                   imu_msg->linear_acceleration.y,
                                   imu_msg->linear_acceleration.z;
    imu_data.angular_velocity << imu_msg->angular_velocity.x,
                                 imu_msg->angular_velocity.y,
                                 imu_msg->angular_velocity.z;

    // 添加到IMU积分器
    imu_integration_->addIMUData(imu_data);
    last_imu_time_ = imu_data.timestamp;

    // 清除旧数据
    imu_integration_->clearOldData(imu_data.timestamp, 10.0);
}

void ROSOdometryNode::publishPose(const Eigen::Matrix4d& pose, const ros::Time& timestamp) {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = timestamp;
    pose_msg.header.frame_id = odom_frame_;

    // 提取位置和姿态
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    extractPose(pose, position, orientation);

    pose_msg.pose.position.x = position.x();
    pose_msg.pose.position.y = position.y();
    pose_msg.pose.position.z = position.z();
    pose_msg.pose.orientation.x = orientation.x();
    pose_msg.pose.orientation.y = orientation.y();
    pose_msg.pose.orientation.z = orientation.z();
    pose_msg.pose.orientation.w = orientation.w();

    pose_pub_.publish(pose_msg);
}

void ROSOdometryNode::publishOdometry(const Eigen::Matrix4d& pose, const ros::Time& timestamp) {
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = timestamp;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = lidar_frame_;

    // 提取位置和姿态
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    extractPose(pose, position, orientation);

    odom_msg.pose.pose.position.x = position.x();
    odom_msg.pose.pose.position.y = position.y();
    odom_msg.pose.pose.position.z = position.z();
    odom_msg.pose.pose.orientation.x = orientation.x();
    odom_msg.pose.pose.orientation.y = orientation.y();
    odom_msg.pose.pose.orientation.z = orientation.z();
    odom_msg.pose.pose.orientation.w = orientation.w();

    // 设置协方差（简化）
    for (int i = 0; i < 36; ++i) {
        odom_msg.pose.covariance[i] = 0.0;
    }
    odom_msg.pose.covariance[0] = 0.01;  // x
    odom_msg.pose.covariance[7] = 0.01;  // y
    odom_msg.pose.covariance[14] = 0.01; // z
    odom_msg.pose.covariance[21] = 0.01; // roll
    odom_msg.pose.covariance[28] = 0.01; // pitch
    odom_msg.pose.covariance[35] = 0.01; // yaw

    odometry_pub_.publish(odom_msg);
}

void ROSOdometryNode::publishPath(const Eigen::Matrix4d& pose, const ros::Time& timestamp) {
    // 更新路径
    path_.header.stamp = timestamp;
    
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = timestamp;
    pose_stamped.header.frame_id = odom_frame_;

    // 提取位置和姿态
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    extractPose(pose, position, orientation);

    pose_stamped.pose.position.x = position.x();
    pose_stamped.pose.position.y = position.y();
    pose_stamped.pose.position.z = position.z();
    pose_stamped.pose.orientation.x = orientation.x();
    pose_stamped.pose.orientation.y = orientation.y();
    pose_stamped.pose.orientation.z = orientation.z();
    pose_stamped.pose.orientation.w = orientation.w();

    path_.poses.push_back(pose_stamped);

    // 限制路径长度
    if (path_.poses.size() > 1000) {
        path_.poses.erase(path_.poses.begin());
    }

    path_pub_.publish(path_);
}

void ROSOdometryNode::publishTransformedCloud(const PointCloudPtr& cloud, 
                                             const Eigen::Matrix4d& pose, 
                                             const ros::Time& timestamp) {
    // 变换点云到里程计坐标系
    PointCloudPtr transformed_cloud(new PointCloud);
    pcl::transformPointCloud(*cloud, *transformed_cloud, pose.cast<float>());

    // 转换为ROS消息
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*transformed_cloud, cloud_msg);
    cloud_msg.header.stamp = timestamp;
    cloud_msg.header.frame_id = odom_frame_;

    transformed_cloud_pub_.publish(cloud_msg);
}

void ROSOdometryNode::publishTransform(const Eigen::Matrix4d& pose, const ros::Time& timestamp) {
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = timestamp;
    transform.header.frame_id = odom_frame_;
    transform.child_frame_id = lidar_frame_;

    // 提取位置和姿态
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    extractPose(pose, position, orientation);

    transform.transform.translation.x = position.x();
    transform.transform.translation.y = position.y();
    transform.transform.translation.z = position.z();
    transform.transform.rotation.x = orientation.x();
    transform.transform.rotation.y = orientation.y();
    transform.transform.rotation.z = orientation.z();
    transform.transform.rotation.w = orientation.w();

    tf_broadcaster_.sendTransform(transform);
}

void ROSOdometryNode::extractPose(const Eigen::Matrix4d& transformation,
                                  Eigen::Vector3d& position,
                                  Eigen::Quaterniond& orientation) {
    position = transformation.block<3, 1>(0, 3);
    Eigen::Matrix3d rotation_matrix = transformation.block<3, 3>(0, 0);
    orientation = Eigen::Quaterniond(rotation_matrix);
}

bool ROSOdometryNode::loadParameters() {
    // 话题参数
    private_nh_.param<std::string>("pointcloud_topic", pointcloud_topic_, "/velodyne_points");
    private_nh_.param<std::string>("imu_topic", imu_topic_, "/imu");
    private_nh_.param<std::string>("pose_topic", pose_topic_, "/lidar_pose");
    private_nh_.param<std::string>("odometry_topic", odometry_topic_, "/lidar_odometry");
    private_nh_.param<std::string>("path_topic", path_topic_, "/lidar_path");
    private_nh_.param<std::string>("transformed_cloud_topic", transformed_cloud_topic_, "/transformed_cloud");

    // 坐标系参数
    private_nh_.param<std::string>("base_frame", base_frame_, "base_link");
    private_nh_.param<std::string>("odom_frame", odom_frame_, "odom");
    private_nh_.param<std::string>("lidar_frame", lidar_frame_, "velodyne");

    // 配准方法
    std::string method_str;
    private_nh_.param<std::string>("registration_method", method_str, "ndt");
    if (method_str == "ndt") {
        registration_method_ = LidarOdometry::RegistrationMethod::NDT;
    } else if (method_str == "gn_icp") {
        registration_method_ = LidarOdometry::RegistrationMethod::GN_ICP;
    } else {
        ROS_ERROR("Unknown registration method: %s", method_str.c_str());
        return false;
    }

    // 处理参数
    private_nh_.param<double>("voxel_size", voxel_size_, 0.1);
    private_nh_.param<double>("max_range", max_range_, 100.0);
    private_nh_.param<double>("min_range", min_range_, 1.0);
    private_nh_.param<bool>("use_imu_prediction", use_imu_prediction_, false);
    private_nh_.param<bool>("publish_transformed_cloud", publish_transformed_cloud_, false);

    ROS_INFO("Parameters loaded:");
    ROS_INFO("  Point cloud topic: %s", pointcloud_topic_.c_str());
    ROS_INFO("  IMU topic: %s", imu_topic_.c_str());
    ROS_INFO("  Registration method: %s", method_str.c_str());
    ROS_INFO("  Voxel size: %.3f", voxel_size_);
    ROS_INFO("  Range: [%.1f, %.1f]", min_range_, max_range_);
    ROS_INFO("  Use IMU prediction: %s", use_imu_prediction_ ? "true" : "false");

    return true;
}

} 
