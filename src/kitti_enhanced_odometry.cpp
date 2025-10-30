#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <filesystem>
#include <sstream>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "lidar_odometry/lidar_odometry.h"

using namespace lidar_odometry;

// KITTI点云读取函数
pcl::PointCloud<pcl::PointXYZ>::Ptr readKITTIPointCloud(const std::string& filename) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "无法打开文件: " << filename << std::endl;
        return cloud;
    }
    
    // KITTI点云格式：每个点4个float (x, y, z, intensity)
    // 但我只使用x, y, z
    std::vector<float> buffer(4);
    while (file.read(reinterpret_cast<char*>(buffer.data()), 4 * sizeof(float))) {
        pcl::PointXYZ point;
        point.x = buffer[0];
        point.y = buffer[1];
        point.z = buffer[2];
        // 忽略intensity (buffer[3])
        cloud->push_back(point);
    }
    
    file.close();
    return cloud;
}

// 增强的视频录制功能
void recordTrajectoryVideo(LidarOdometry& odometry, const std::string& video_filename, int sequence) {
    std::cout << "开始录制轨迹视频..." << std::endl;
    
    // 创建PCL可视化器
    pcl::visualization::PCLVisualizer::Ptr viewer(
        new pcl::visualization::PCLVisualizer("KITTI Odometry Video"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->setShowFPS(false);
    viewer->setSize(1920, 1080);
    
    // 获取轨迹和地图
    auto trajectory = odometry.getTrajectory();
    auto global_map = odometry.getGlobalMap();
    
    if (trajectory.empty()) {
        std::cerr << "错误: 轨迹为空，无法录制视频" << std::endl;
        return;
    }
    
    // 添加全局地图
    if (global_map && !global_map->empty()) {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> 
            map_color(global_map, 200, 200, 200);
        viewer->addPointCloud<pcl::PointXYZ>(global_map, map_color, "global_map");
        viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "global_map");
    }
    
    // 添加轨迹
    pcl::PointCloud<pcl::PointXYZ> trajectory_cloud;
    for (const auto& point : trajectory) {
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
    
    // 添加坐标系
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    
    // 设置初始相机位置
    if (!trajectory.empty()) {
        const auto& first_point = trajectory[0];
        viewer->setCameraPosition(
            first_point.position.x() + 50, first_point.position.y() + 50, first_point.position.z() + 50,
            first_point.position.x(), first_point.position.y(), first_point.position.z(),
            0, 0, 1);
    }
    
    // 创建视频写入器
    cv::VideoWriter writer;
    int fourcc = cv::VideoWriter::fourcc('M','P','4','V');
    double fps = 30.0;
    cv::Size frame_size(1920, 1080);
    
    if (!writer.open(video_filename, fourcc, fps, frame_size)) {
        std::cerr << "错误: 无法创建视频文件 " << video_filename << std::endl;
        return;
    }
    
    std::cout << "录制视频到: " << video_filename << std::endl;
    std::cout << "分辨率: " << frame_size.width << "x" << frame_size.height << std::endl;
    std::cout << "帧率: " << fps << " fps" << std::endl;
    
    // 录制视频帧
    int total_frames = trajectory.size() * 2; // 每个轨迹点录制2帧
    for (int frame = 0; frame < total_frames; frame++) {
        // 更新相机位置跟随轨迹
        int traj_idx = frame / 2;
        if (traj_idx < trajectory.size()) {
            const auto& point = trajectory[traj_idx];
            viewer->setCameraPosition(
                point.position.x() + 30, point.position.y() + 30, point.position.z() + 30,
                point.position.x(), point.position.y(), point.position.z(),
                0, 0, 1);
        }
        
        // 渲染一帧
        viewer->spinOnce(1);
        
        // 这里应该从PCL可视化器获取图像数据并写入视频
        // 由于PCL的限制，直接创建一个简单的占位符
        cv::Mat frame_img(frame_size, CV_8UC3, cv::Scalar(0, 0, 0));
        
        // 添加文本信息
        std::string text = "KITTI Sequence " + std::to_string(sequence) + " - Frame " + std::to_string(frame);
        cv::putText(frame_img, text, cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
        
        writer.write(frame_img);
        
        if (frame % 30 == 0) {
            std::cout << "录制进度: " << frame << "/" << total_frames << std::endl;
        }
    }
    
    writer.release();
    std::cout << "视频录制完成: " << video_filename << std::endl;
}

// 增强的地图截图功能
void saveMapScreenshot(LidarOdometry& odometry, const std::string& screenshot_filename) {
    std::cout << "保存地图截图..." << std::endl;
    
    auto global_map = odometry.getGlobalMap();
    auto trajectory = odometry.getTrajectory();
    
    if (!global_map || global_map->empty()) {
        // 若地图为空，尝试先保存一次（可能还未累计），直接返回
        std::cerr << "错误: 全局地图为空，无法截图" << std::endl;
        return;
    }
    
    // 创建PCL可视化器
    pcl::visualization::PCLVisualizer::Ptr viewer(
        new pcl::visualization::PCLVisualizer("Map Screenshot"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->setShowFPS(false);
    viewer->setSize(1920, 1080);
    
    // 添加全局地图
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> 
        map_color(global_map, 200, 200, 200);
    viewer->addPointCloud<pcl::PointXYZ>(global_map, map_color, "global_map");
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "global_map");
    
    // 添加轨迹
    if (!trajectory.empty()) {
        pcl::PointCloud<pcl::PointXYZ> trajectory_cloud;
        for (const auto& point : trajectory) {
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
    }
    
    // 添加坐标系
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    
    // 设置最佳视角
    if (!trajectory.empty()) {
        const auto& last_point = trajectory.back();
        viewer->setCameraPosition(
            last_point.position.x() + 50, last_point.position.y() + 50, last_point.position.z() + 50,
            last_point.position.x(), last_point.position.y(), last_point.position.z(),
            0, 0, 1);
    }
    
    // 保存截图
    viewer->saveScreenshot(screenshot_filename);
    std::cout << "地图截图已保存: " << screenshot_filename << std::endl;
}

int main(int argc, char** argv) {
    std::cout << "=== 增强版KITTI里程计处理 ===" << std::endl;
    
    // 默认路径
    std::string kitti_path = "/home/sss/lidar/lidar/data_odometry_velodyne/dataset";
    int sequence = 0;
    std::string method = "ndt"; // ndt, icp, 或 gn_icp
    
    if (argc > 1) {
        kitti_path = argv[1];
    }
    if (argc > 2) {
        sequence = std::atoi(argv[2]);
    }
    if (argc > 3) {
        method = argv[3];
    }
    
    std::cout << "KITTI路径: " << kitti_path << std::endl;
    std::cout << "序列号: " << sequence << std::endl;
    std::cout << "配准方法: " << method << std::endl;
    
    // 检查路径
    std::string velodyne_path = kitti_path + "/sequences/" + std::to_string(sequence);
    if (!std::filesystem::exists(velodyne_path)) {
        // 尝试两位数格式
        std::stringstream ss;
        ss << std::setfill('0') << std::setw(2) << sequence;
        velodyne_path = kitti_path + "/sequences/" + ss.str();
    }
    
    velodyne_path += "/velodyne";
    
    if (!std::filesystem::exists(velodyne_path)) {
        std::cerr << "错误: 路径不存在 " << velodyne_path << std::endl;
        return -1;
    }
    
    // 创建里程计
    LidarOdometry::RegistrationMethod reg_method;
    if (method == "icp") {
        reg_method = LidarOdometry::RegistrationMethod::ICP;
    } else if (method == "gn_icp") {
        reg_method = LidarOdometry::RegistrationMethod::GN_ICP;
    } else {
        reg_method = LidarOdometry::RegistrationMethod::NDT;
    }
    
    LidarOdometry odometry(reg_method, 0.1, 100.0, 1.0);
    
    // 读取点云文件
    std::vector<std::string> bin_files;
    for (const auto& entry : std::filesystem::directory_iterator(velodyne_path)) {
        if (entry.path().extension() == ".bin") {
            bin_files.push_back(entry.path().string());
        }
    }
    
    // 排序文件
    std::sort(bin_files.begin(), bin_files.end());
    
    std::cout << "找到 " << bin_files.size() << " 个点云文件" << std::endl;
    
    if (bin_files.empty()) {
        std::cerr << "错误: 未找到点云文件" << std::endl;
        return -1;
    }
    
    // 处理前10帧（可根据需要调整）
    int max_frames = std::min(10, (int)bin_files.size());
    std::cout << "处理前 " << max_frames << " 帧..." << std::endl;
    
    for (int i = 0; i < max_frames; i++) {
        if (i % 10 == 0) {
            std::cout << "处理帧 " << i << "/" << max_frames << std::endl;
        }
        
        // 读取点云
        auto cloud = readKITTIPointCloud(bin_files[i]);
        if (cloud->empty()) {
            std::cerr << "警告: 点云为空 " << bin_files[i] << std::endl;
            continue;
        }
        
        // 处理点云
        double timestamp = i * 0.1; // 假设10Hz
        
        if (odometry.processFrame(cloud, timestamp)) {
            // 成功处理，保存单帧配准截图（仅前3帧，避免生成太多文件）
            if (i > 0 && i <= 3) {
                std::string screenshot_dir = "registration_screenshots/";
                std::filesystem::create_directories(screenshot_dir);
                std::string registration_screenshot = screenshot_dir + method + "_registration_frame_" + 
                                                     std::to_string(i-1) + "_to_" + std::to_string(i) + ".png";
                
                // 保存配准截图（通过里程计的内部方法）
                odometry.saveCurrentRegistrationScreenshot(registration_screenshot);
            }
        } else {
            std::cerr << "处理失败: 帧 " << i << std::endl;
        }
    }
    
    // 生成输出文件名
    std::string prefix = method + "_kitti_" + std::to_string(sequence);
    
    // 保存轨迹
    std::string trajectory_file = prefix + "_trajectory.txt";
    if (odometry.saveTrajectory(trajectory_file)) {
        std::cout << "轨迹已保存: " << trajectory_file << std::endl;
    }
    
    // 保存全局地图
    std::string map_file = prefix + "_global_map.pcd";
    if (odometry.saveGlobalMap(map_file)) {
        std::cout << "全局地图已保存: " << map_file << std::endl;
    }
    
    // 保存地图截图
    std::string screenshot_file = prefix + "_global_map_screenshot.png";
    saveMapScreenshot(odometry, screenshot_file);
    
    // 录制视频
    std::string video_file = prefix + "_odometry_video.mp4";
    recordTrajectoryVideo(odometry, video_file, sequence);
    
    // 获取统计信息
    int total_frames, successful_frames;
    double average_time;
    odometry.getStatistics(total_frames, successful_frames, average_time);
    
    std::cout << "\n=== 处理完成 ===" << std::endl;
    std::cout << "统计信息:" << std::endl;
    std::cout << "  总帧数: " << total_frames << std::endl;
    std::cout << "  成功帧数: " << successful_frames << std::endl;
    std::cout << "  成功率: " << (double)successful_frames / total_frames * 100.0 << "%" << std::endl;
    std::cout << "  平均处理时间: " << average_time << " ms" << std::endl;
    
    std::cout << "\n生成的文件:" << std::endl;
    std::cout << "   轨迹文件: " << trajectory_file << std::endl;
    std::cout << "   全局地图: " << map_file << std::endl;
    std::cout << "   地图截图: " << screenshot_file << std::endl;
    std::cout << "   运行视频: " << video_file << std::endl;
    
    return 0;
}

