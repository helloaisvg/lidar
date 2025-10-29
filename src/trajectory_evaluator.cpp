#include <ros/ros.h>
#include <iostream>
#include <string>
#include "lidar_odometry/trajectory_evaluator.h"

using namespace lidar_odometry;

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_evaluator");
    ros::NodeHandle nh;
    
    std::cout << "=== Trajectory Evaluator ===" << std::endl;
    
    if (argc < 3) {
        std::cout << "Usage: " << argv[0] << " <estimated_trajectory> <ground_truth_trajectory> [output_file]" << std::endl;
        return -1;
    }
    
    std::string estimated_file = argv[1];
    std::string ground_truth_file = argv[2];
    std::string output_file = "evaluation_result.txt";
    
    if (argc > 3) {
        output_file = argv[3];
    }
    
    // 创建评估器
    TrajectoryEvaluator evaluator;
    
    // 加载轨迹
    std::cout << "Loading estimated trajectory: " << estimated_file << std::endl;
    if (!evaluator.loadEstimatedTrajectory(estimated_file)) {
        std::cerr << "Failed to load estimated trajectory!" << std::endl;
        return -1;
    }
    
    std::cout << "Loading ground truth trajectory: " << ground_truth_file << std::endl;
    if (!evaluator.loadGroundTruthTrajectory(ground_truth_file)) {
        std::cerr << "Failed to load ground truth trajectory!" << std::endl;
        return -1;
    }
    
    // 执行评估
    std::cout << "\nComputing evaluation metrics..." << std::endl;
    EvaluationResult result = evaluator.evaluate(1.0, 1.0); // 1秒间隔的RPE
    
    // 打印结果
    evaluator.printEvaluationResult(result);
    
    // 保存结果
    if (evaluator.saveEvaluationResult(output_file, result)) {
        std::cout << "\nEvaluation results saved to: " << output_file << std::endl;
    }
    
    return 0;
}