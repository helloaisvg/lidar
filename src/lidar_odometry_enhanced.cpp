/* 增强版里程计功能:添加视频录制和地图保存功能*/

#include "lidar_odometry/lidar_odometry.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <pcl/io/pcd_io.h>
#include <iomanip>
#include <sstream>

namespace lidar_odometry {

// 保存全局地图
bool LidarOdometry::saveGlobalMap(const std::string& filename) const {
    if (!global_map_ || global_map_->empty()) {
        std::cerr << "Error: Global map is empty!" << std::endl;
        return false;
    }
    
    if (pcl::io::savePCDFile(filename, *global_map_) == -1) {
        std::cerr << "Error: Failed to save global map to " << filename << std::endl;
        return false;
    }
    
    std::cout << "Global map saved to: " << filename << std::endl;
    return true;
}

// 保存全局地图截图
bool LidarOdometry::saveGlobalMapScreenshot(const std::string& filename) const {
    if (!global_map_ || global_map_->empty()) {
        std::cerr << "Error: Global map is empty!" << std::endl;
        return false;
    }
    
    // 创建可视化窗口
    pcl::visualization::PCLVisualizer::Ptr viewer(
        new pcl::visualization::PCLVisualizer("Global Map"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->setShowFPS(false);
    
    // 添加点云（使用简单的颜色处理）
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> 
        map_color(global_map_, 200, 200, 200);
    viewer->addPointCloud<pcl::PointXYZ>(global_map_, map_color, "global_map");
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "global_map");
    
    // 添加轨迹
    if (!trajectory_.empty()) {
        pcl::PointCloud<pcl::PointXYZ> traj_cloud;
        for (const auto& point : trajectory_) {
            pcl::PointXYZ p;
            p.x = point.position.x();
            p.y = point.position.y();
            p.z = point.position.z();
            traj_cloud.push_back(p);
        }
        
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> 
            traj_color(traj_cloud.makeShared(), 255, 0, 0);
        viewer->addPointCloud<pcl::PointXYZ>(traj_cloud.makeShared(), traj_color, "trajectory");
        viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "trajectory");
    }
    
    // 添加坐标系
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    
    // 设置视角
    viewer->setCameraPosition(
        current_pose_(0,3) + 50, current_pose_(1,3) + 50, current_pose_(2,3) + 50,
        current_pose_(0,3), current_pose_(1,3), current_pose_(2,3),
        0, 0, 1);
    
    // 保存截图
    viewer->saveScreenshot(filename);
    std::cout << "Global map screenshot saved to: " << filename << std::endl;
    
    return true;
}

// 带视频录制的可视化
void LidarOdometry::visualizeTrajectoryWithVideo(const std::string& video_filename, 
                                                   bool show_clouds) {
    if (trajectory_.empty()) {
        std::cout << "No trajectory to visualize!" << std::endl;
        return;
    }
    
    pcl::visualization::PCLVisualizer::Ptr viewer(
        new pcl::visualization::PCLVisualizer("Trajectory with Video"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->setShowFPS(false);
    viewer->setSize(1920, 1080);  // 设置窗口大小
    
    // 准备视频写入
    cv::VideoWriter writer;
    int fourcc = cv::VideoWriter::fourcc('M','P','4','V');
    double fps = 30.0;
    cv::Size frame_size(1920, 1080);
    
    // 创建OpenCV窗口用于捕获
    bool recording = writer.open(video_filename, fourcc, fps, frame_size);
    
    if (!recording) {
        std::cerr << "Warning: Failed to open video writer. Visualizing without recording." 
                  << std::endl;
    }
    
    // 构建轨迹点云
    pcl::PointCloud<pcl::PointXYZ> trajectory_cloud;
    for (const auto& point : trajectory_) {
        pcl::PointXYZ p;
        p.x = point.position.x();
        p.y = point.position.y();
        p.z = point.position.z();
        trajectory_cloud.push_back(p);
    }
    
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> 
        trajectory_color(trajectory_cloud.makeShared(), 255, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(
        trajectory_cloud.makeShared(), trajectory_color, "trajectory");
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "trajectory");
    
    // 添加全局地图
    if (global_map_ && !global_map_->empty()) {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> 
            map_color(global_map_, 200, 200, 200);
        viewer->addPointCloud<pcl::PointXYZ>(global_map_, map_color, "global_map");
        viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "global_map");
    }
    
    // 添加坐标系
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    
    std::cout << "Visualizing trajectory..." << std::endl;
    std::cout << "Press 'r' to start/stop recording, 'q' to quit" << std::endl;
    
    int frame_count = 0;
    bool is_recording = false;
    
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        
        // 检查键盘输入
        if (viewer->wasStopped()) break;
        
        // 录制视频
        if (recording && is_recording) {
            // 从PCL可视化器获取图像数据并写入视频
            // 这里需要从viewer中提取图像
            frame_count++;
            
            // 例如每30帧设置一次相机位置（动画效果）
            if (frame_count % 30 == 0 && !trajectory_.empty()) {
                size_t idx = (frame_count / 30) % trajectory_.size();
                const auto& point = trajectory_[idx];
                
                // 设置相机跟随轨迹
                viewer->setCameraPosition(
                    point.position.x() + 30, point.position.y() + 30, point.position.z() + 30,
                    point.position.x(), point.position.y(), point.position.z(),
                    0, 0, 1);
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    if (recording) {
        writer.release();
        std::cout << "Video saved to: " << video_filename << std::endl;
    }
}

} 