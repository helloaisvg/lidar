#include <iostream>
#include <string>
#include "lidar_odometry/lidar_odometry.h"

using namespace lidar_odometry;

int main(int argc, char** argv) {
    std::string kitti_path = "/home/sss/lidar/lidar/data_odometry_velodyne/dataset";
    int sequence = 0;
    std::string method = "ndt"; // ndt, icp, or gn_icp

    if (argc > 1) kitti_path = argv[1];
    if (argc > 2) sequence = std::atoi(argv[2]);
    if (argc > 3) method = argv[3];

    LidarOdometry::RegistrationMethod reg;
    if (method == "icp") {
        reg = LidarOdometry::RegistrationMethod::ICP;
    } else if (method == "gn_icp") {
        reg = LidarOdometry::RegistrationMethod::GN_ICP;
    } else {
        reg = LidarOdometry::RegistrationMethod::NDT;
    }

    LidarOdometry odom(reg, 0.1, 100.0, 1.0);

    std::cout << "Processing KITTI: path=" << kitti_path << ", seq=" << sequence
              << ", method=" << method << std::endl;

    if (!odom.processKITTISequence(kitti_path, sequence)) {
        std::cerr << "Failed to process sequence" << std::endl;
        return 1;
    }

    std::string prefix = method + "_kitti_" + std::to_string(sequence);
    odom.saveTrajectory(prefix + "_trajectory.txt");
    odom.saveGlobalMap(prefix + "_global_map.pcd");
    odom.saveGlobalMapScreenshot(prefix + "_global_map_screenshot.png");

    std::cout << "Done." << std::endl;
    return 0;
}


