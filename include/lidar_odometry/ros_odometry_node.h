#ifndef ROS_ODOMETRY_NODE_H
#define ROS_ODOMETRY_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <memory>
#include <string>

#include "lidar_odometry.h"
#include "imu_integration.h"

namespace lidar_odometry {

/**
 * @brief ROS里程计节点类
 * 实现ROS节点用于激光雷达里程计
 */
class ROSOdometryNode {
public:
    using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
    using PointCloudPtr = PointCloud::Ptr;

    /**
     * @brief 构造函数
     * @param nh ROS节点句柄
     */
    explicit ROSOdometryNode(ros::NodeHandle& nh);

    /**
     * @brief 析构函数
     */
    ~ROSOdometryNode() = default;

    /**
     * @brief 初始化节点
     * @return 是否成功初始化
     */
    bool initialize();

    /**
     * @brief 运行节点
     */
    void run();

    /**
     * @brief 停止节点
     */
    void stop();

private:
    ros::NodeHandle& nh_;                    // ROS节点句柄
    ros::NodeHandle private_nh_;            // 私有节点句柄
    
    // 订阅者
    ros::Subscriber pointcloud_sub_;        // 点云订阅者
    ros::Subscriber imu_sub_;               // IMU订阅者
    
    // 发布者
    ros::Publisher pose_pub_;                // 位姿发布者
    ros::Publisher odometry_pub_;            // 里程计发布者
    ros::Publisher path_pub_;                // 路径发布者
    ros::Publisher transformed_cloud_pub_;   // 变换后点云发布者
    
    // TF广播器
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    
    // 里程计和IMU积分器
    std::unique_ptr<LidarOdometry> odometry_;
    std::unique_ptr<IMUIntegration> imu_integration_;
    
    // 路径
    nav_msgs::Path path_;
    
    // 参数
    std::string pointcloud_topic_;          // 点云话题
    std::string imu_topic_;                  // IMU话题
    std::string pose_topic_;                 // 位姿话题
    std::string odometry_topic_;             // 里程计话题
    std::string path_topic_;                 // 路径话题
    std::string transformed_cloud_topic_;     // 变换后点云话题
    std::string base_frame_;                 // 基础坐标系
    std::string odom_frame_;                 // 里程计坐标系
    std::string lidar_frame_;                // 激光雷达坐标系
    
    // 配准方法
    LidarOdometry::RegistrationMethod registration_method_;
    
    // 处理参数
    double voxel_size_;                      // 体素大小
    double max_range_;                       // 最大距离
    double min_range_;                       // 最小距离
    bool use_imu_prediction_;               // 是否使用IMU预测
    bool publish_transformed_cloud_;         // 是否发布变换后点云
    
    // 状态
    bool is_initialized_;                   // 是否已初始化
    bool is_running_;                       // 是否正在运行
    double last_cloud_time_;                // 最后点云时间
    double last_imu_time_;                   // 最后IMU时间
    
    /**
     * @brief 点云回调函数
     * @param cloud_msg 点云消息
     */
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);

    /**
     * @brief IMU回调函数
     * @param imu_msg IMU消息
     */
    void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg);

    /**
     * @brief 发布位姿
     * @param pose 位姿变换矩阵
     * @param timestamp 时间戳
     */
    void publishPose(const Eigen::Matrix4d& pose, const ros::Time& timestamp);

    /**
     * @brief 发布里程计
     * @param pose 位姿变换矩阵
     * @param timestamp 时间戳
     */
    void publishOdometry(const Eigen::Matrix4d& pose, const ros::Time& timestamp);

    /**
     * @brief 发布路径
     * @param pose 位姿变换矩阵
     * @param timestamp 时间戳
     */
    void publishPath(const Eigen::Matrix4d& pose, const ros::Time& timestamp);

    /**
     * @brief 发布变换后点云
     * @param cloud 点云
     * @param pose 位姿变换矩阵
     * @param timestamp 时间戳
     */
    void publishTransformedCloud(const PointCloudPtr& cloud, 
                                const Eigen::Matrix4d& pose, 
                                const ros::Time& timestamp);

    /**
     * @brief 发布TF变换
     * @param pose 位姿变换矩阵
     * @param timestamp 时间戳
     */
    void publishTransform(const Eigen::Matrix4d& pose, const ros::Time& timestamp);

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
     * @brief 加载参数
     * @return 是否成功加载参数
     */
    bool loadParameters();
};

} 

#endif 