#include <ros/ros.h>
#include <iostream>
#include <string>
#include "lidar_odometry/lidar_odometry.h"

using namespace lidar_odometry;

int main(int argc, char** argv) {
    ros::init(argc, argv, "icp_odometry");
    ros::NodeHandle nh;
    
    std::cout << "=== GN-ICP Odometry Demo ===" << std::endl;
    
    // 创建GN-ICP里程计
    LidarOdometry odometry(LidarOdometry::RegistrationMethod::GN_ICP, 0.1, 100.0, 1.0);
    
    // 处理KITTI数据集
    std::string data_path = "/path/to/kitti/dataset";
    int sequence = 0;
    
    if (argc > 1) {
        data_path = argv[1];
    }
    if (argc > 2) {
        sequence = std::atoi(argv[2]);
    }
    
    std::cout << "Processing KITTI sequence " << sequence << " from: " << data_path << std::endl;
    
    // 处理序列
    if (odometry.processKITTISequence(data_path, sequence)) {
        std::cout << "KITTI sequence processing completed!" << std::endl;
        
        // 保存轨迹
        std::string trajectory_file = "gn_icp_trajectory_" + std::to_string(sequence) + ".txt";
        if (odometry.saveTrajectory(trajectory_file)) {
            std::cout << "Trajectory saved to: " << trajectory_file << std::endl;
        }
        
        // 获取统计信息
        int total_frames, successful_frames;
        double average_time;
        odometry.getStatistics(total_frames, successful_frames, average_time);
        
        std::cout << "\nStatistics:" << std::endl;
        std::cout << "  Total frames: " << total_frames << std::endl;
        std::cout << "  Successful frames: " << successful_frames << std::endl;
        std::cout << "  Success rate: " << (double)successful_frames / total_frames * 100.0 << "%" << std::endl;
        std::cout << "  Average processing time: " << average_time << " ms" << std::endl;
        
        // 可视化轨迹
        std::cout << "\nVisualizing trajectory..." << std::endl;
        odometry.visualizeTrajectory(false);
    } else {
        std::cerr << "Failed to process KITTI sequence!" << std::endl;
        return -1;
    }
    
    return 0;
}
