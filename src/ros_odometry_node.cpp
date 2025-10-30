#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <csignal>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <memory>
#include <string>

#include "lidar_odometry/lidar_odometry.h"
#include "lidar_odometry/imu_integration.h"

using namespace lidar_odometry;
using std::placeholders::_1;

class ROSOdometryNode : public rclcpp::Node {
public:
    using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
    using PointCloudPtr = PointCloud::Ptr;

    ROSOdometryNode() : Node("ros_odometry_node") {
        // 声明参数
        this->declare_parameter<std::string>("pointcloud_topic", "/velodyne_points");
        this->declare_parameter<std::string>("imu_topic", "/imu");
        this->declare_parameter<std::string>("pose_topic", "/lidar_pose");
        this->declare_parameter<std::string>("odometry_topic", "/lidar_odometry");
        this->declare_parameter<std::string>("path_topic", "/lidar_path");
        this->declare_parameter<std::string>("transformed_cloud_topic", "/transformed_cloud");
        this->declare_parameter<std::string>("base_frame", "base_link");
        this->declare_parameter<std::string>("odom_frame", "odom");
        this->declare_parameter<std::string>("lidar_frame", "velodyne");
        this->declare_parameter<std::string>("registration_method", "ndt");
        this->declare_parameter<double>("voxel_size", 0.1);
        this->declare_parameter<double>("max_range", 100.0);
        this->declare_parameter<double>("min_range", 1.0);
        this->declare_parameter<bool>("use_imu_prediction", false);
        this->declare_parameter<bool>("publish_transformed_cloud", false);
        this->declare_parameter<std::string>("trajectory_output_file", "ros_odometry_trajectory.txt");

        // 获取参数
        std::string pointcloud_topic = this->get_parameter("pointcloud_topic").as_string();
        std::string imu_topic = this->get_parameter("imu_topic").as_string();
        std::string pose_topic = this->get_parameter("pose_topic").as_string();
        std::string odometry_topic = this->get_parameter("odometry_topic").as_string();
        std::string path_topic = this->get_parameter("path_topic").as_string();
        std::string transformed_cloud_topic = this->get_parameter("transformed_cloud_topic").as_string();
        odom_frame_ = this->get_parameter("odom_frame").as_string();
        lidar_frame_ = this->get_parameter("lidar_frame").as_string();
        std::string method_str = this->get_parameter("registration_method").as_string();
        double voxel_size = this->get_parameter("voxel_size").as_double();
        double max_range = this->get_parameter("max_range").as_double();
        double min_range = this->get_parameter("min_range").as_double();
        use_imu_prediction_ = this->get_parameter("use_imu_prediction").as_bool();
        publish_transformed_cloud_ = this->get_parameter("publish_transformed_cloud").as_bool();
        trajectory_output_file_ = this->get_parameter("trajectory_output_file").as_string();

        // 设置配准方法
        LidarOdometry::RegistrationMethod reg_method;
        if (method_str == "ndt") {
            reg_method = LidarOdometry::RegistrationMethod::NDT;
        } else if (method_str == "icp") {
            reg_method = LidarOdometry::RegistrationMethod::ICP;
        } else if (method_str == "gn_icp") {
            reg_method = LidarOdometry::RegistrationMethod::GN_ICP;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Unknown registration method: %s", method_str.c_str());
            return;
        }

        // 创建里程计
        odometry_ = std::make_unique<LidarOdometry>(reg_method, voxel_size, max_range, min_range);
        imu_integration_ = std::make_unique<IMUIntegration>();

        // 初始化路径
        path_msg_.header.frame_id = odom_frame_;

        // 创建订阅者
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            pointcloud_topic, 10, std::bind(&ROSOdometryNode::pointCloudCallback, this, _1));
        
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, 100, std::bind(&ROSOdometryNode::imuCallback, this, _1));

        // 创建发布者
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic, 10);
        odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odometry_topic, 10);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>(path_topic, 10);
        
        if (publish_transformed_cloud_) {
            transformed_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                transformed_cloud_topic, 10);
        }

        // TF广播器
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        RCLCPP_INFO(this->get_logger(), "ROS2 Odometry Node initialized!");
        RCLCPP_INFO(this->get_logger(), "  Point cloud topic: %s", pointcloud_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  Registration method: %s", method_str.c_str());
        RCLCPP_INFO(this->get_logger(), "  Trajectory output: %s", trajectory_output_file_.c_str());

    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
        // 转换点云格式
        PointCloudPtr cloud(new PointCloud);
        pcl::fromROSMsg(*cloud_msg, *cloud);

        if (cloud->empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty point cloud!");
            return;
        }

        // 获取时间戳
        double timestamp = rclcpp::Time(cloud_msg->header.stamp).seconds();

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
            RCLCPP_WARN(this->get_logger(), "Failed to process point cloud frame!");
        }
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg) {
        // 转换IMU数据
        IMUData imu_data;
        imu_data.timestamp = rclcpp::Time(imu_msg->header.stamp).seconds();
        imu_data.linear_acceleration << imu_msg->linear_acceleration.x,
                                       imu_msg->linear_acceleration.y,
                                       imu_msg->linear_acceleration.z;
        imu_data.angular_velocity << imu_msg->angular_velocity.x,
                                    imu_msg->angular_velocity.y,
                                    imu_msg->angular_velocity.z;

        // 添加到IMU积分器
        imu_integration_->addIMUData(imu_data);
        imu_integration_->clearOldData(imu_data.timestamp, 10.0);
    }

    void publishPose(const Eigen::Matrix4d& pose, const rclcpp::Time& timestamp) {
        auto pose_msg = geometry_msgs::msg::PoseStamped();
        pose_msg.header.stamp = timestamp;
        pose_msg.header.frame_id = odom_frame_;

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

        pose_pub_->publish(pose_msg);
    }

    void publishOdometry(const Eigen::Matrix4d& pose, const rclcpp::Time& timestamp) {
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = timestamp;
        odom_msg.header.frame_id = odom_frame_;
        odom_msg.child_frame_id = lidar_frame_;

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

        odometry_pub_->publish(odom_msg);
    }

    void publishPath(const Eigen::Matrix4d& pose, const rclcpp::Time& timestamp) {
        path_msg_.header.stamp = timestamp;
        
        auto pose_stamped = geometry_msgs::msg::PoseStamped();
        pose_stamped.header.stamp = timestamp;
        pose_stamped.header.frame_id = odom_frame_;

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

        path_msg_.poses.push_back(pose_stamped);

        // 限制路径长度
        if (path_msg_.poses.size() > 1000) {
            path_msg_.poses.erase(path_msg_.poses.begin());
        }

        path_pub_->publish(path_msg_);
    }

    void publishTransformedCloud(const PointCloudPtr& cloud, 
                                const Eigen::Matrix4d& pose, 
                                const rclcpp::Time& timestamp) {
        PointCloudPtr transformed_cloud(new PointCloud);
        pcl::transformPointCloud(*cloud, *transformed_cloud, pose.cast<float>());

        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*transformed_cloud, cloud_msg);
        cloud_msg.header.stamp = timestamp;
        cloud_msg.header.frame_id = odom_frame_;

        transformed_cloud_pub_->publish(cloud_msg);
    }

    void publishTransform(const Eigen::Matrix4d& pose, const rclcpp::Time& timestamp) {
        auto transform = geometry_msgs::msg::TransformStamped();
        transform.header.stamp = timestamp;
        transform.header.frame_id = odom_frame_;
        transform.child_frame_id = lidar_frame_;

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

        tf_broadcaster_->sendTransform(transform);
    }

    void extractPose(const Eigen::Matrix4d& transformation,
                     Eigen::Vector3d& position,
                     Eigen::Quaterniond& orientation) {
        position = transformation.block<3, 1>(0, 3);
        Eigen::Matrix3d rotation_matrix = transformation.block<3, 3>(0, 0);
        orientation = Eigen::Quaterniond(rotation_matrix);
    }

    // 订阅者
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    
    // 发布者
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr transformed_cloud_pub_;
    
    // TF广播器
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // 里程计和IMU积分器
    std::unique_ptr<LidarOdometry> odometry_;
    std::unique_ptr<IMUIntegration> imu_integration_;
    
    // 路径
    nav_msgs::msg::Path path_msg_;
    
    // 参数
    std::string odom_frame_;
    std::string lidar_frame_;
    bool use_imu_prediction_;
    bool publish_transformed_cloud_;
    std::string trajectory_output_file_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ROSOdometryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}