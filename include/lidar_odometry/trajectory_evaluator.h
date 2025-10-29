#ifndef TRAJECTORY_EVALUATOR_H
#define TRAJECTORY_EVALUATOR_H

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <cmath>

namespace lidar_odometry {

/**
 * @brief 轨迹点结构
 */
struct TrajectoryPoint {
    double timestamp;                    ///< 时间戳
    Eigen::Vector3d position;           ///< 位置
    Eigen::Quaterniond orientation;     ///< 姿态四元数
    
    TrajectoryPoint() : timestamp(0.0) {}
    
    TrajectoryPoint(double t, const Eigen::Vector3d& pos, const Eigen::Quaterniond& quat)
        : timestamp(t), position(pos), orientation(quat) {}
};

/**
 * @brief 评估结果结构
 */
struct EvaluationResult {
    double ate_rmse;                    ///< 绝对轨迹误差RMSE
    double ate_mean;                    ///< 绝对轨迹误差均值
    double ate_std;                     ///< 绝对轨迹误差标准差
    double ate_max;                     ///< 绝对轨迹误差最大值
    
    double rpe_trans_rmse;             ///< 相对位姿误差平移RMSE
    double rpe_rot_rmse;               ///< 相对位姿误差旋转RMSE
    double rpe_trans_mean;             ///< 相对位姿误差平移均值
    double rpe_rot_mean;               ///< 相对位姿误差旋转均值
    
    double trajectory_length;           ///< 轨迹长度
    double average_speed;               ///< 平均速度
    
    EvaluationResult() 
        : ate_rmse(0.0), ate_mean(0.0), ate_std(0.0), ate_max(0.0)
        , rpe_trans_rmse(0.0), rpe_rot_rmse(0.0), rpe_trans_mean(0.0), rpe_rot_mean(0.0)
        , trajectory_length(0.0), average_speed(0.0) {}
};

/**
 * @brief 轨迹评估器类
 * 实现轨迹评估功能，支持ATE和RPE计算
 */
class TrajectoryEvaluator {
public:
    /**
     * @brief 构造函数
     */
    TrajectoryEvaluator();

    /**
     * @brief 析构函数
     */
    ~TrajectoryEvaluator() = default;

    /**
     * @brief 加载估计轨迹
     * @param filename 轨迹文件名
     * @return 是否成功加载
     */
    bool loadEstimatedTrajectory(const std::string& filename);

    /**
     * @brief 加载真实轨迹
     * @param filename 轨迹文件名
     * @return 是否成功加载
     */
    bool loadGroundTruthTrajectory(const std::string& filename);

    /**
     * @brief 设置估计轨迹
     * @param trajectory 轨迹点列表
     */
    void setEstimatedTrajectory(const std::vector<TrajectoryPoint>& trajectory);

    /**
     * @brief 设置真实轨迹
     * @param trajectory 轨迹点列表
     */
    void setGroundTruthTrajectory(const std::vector<TrajectoryPoint>& trajectory);

    /**
     * @brief 计算绝对轨迹误差(ATE)
     * @return 评估结果
     */
    EvaluationResult computeATE();

    /**
     * @brief 计算相对位姿误差(RPE)
     * @param delta 时间间隔
     * @param delta_unit 时间单位
     * @return 评估结果
     */
    EvaluationResult computeRPE(double delta = 1.0, double delta_unit = 1.0);

    /**
     * @brief 计算完整评估
     * @param delta RPE时间间隔
     * @param delta_unit RPE时间单位
     * @return 评估结果
     */
    EvaluationResult evaluate(double delta = 1.0, double delta_unit = 1.0);

    /**
     * @brief 保存评估结果
     * @param filename 输出文件名
     * @param result 评估结果
     * @return 是否成功保存
     */
    bool saveEvaluationResult(const std::string& filename, const EvaluationResult& result);

    /**
     * @brief 打印评估结果
     * @param result 评估结果
     */
    void printEvaluationResult(const EvaluationResult& result);

    /**
     * @brief 可视化轨迹对比
     * @param output_filename 输出图像文件名
     * @return 是否成功生成图像
     */
    bool visualizeTrajectoryComparison(const std::string& output_filename = "trajectory_comparison.png");

    /**
     * @brief 获取估计轨迹
     * @return 估计轨迹
     */
    const std::vector<TrajectoryPoint>& getEstimatedTrajectory() const;

    /**
     * @brief 获取真实轨迹
     * @return 真实轨迹
     */
    const std::vector<TrajectoryPoint>& getGroundTruthTrajectory() const;

private:
    std::vector<TrajectoryPoint> estimated_trajectory_;   // 估计轨迹
    std::vector<TrajectoryPoint> ground_truth_trajectory_; // 真实轨迹
    
    /**
     * @brief 从文件加载轨迹
     * @param filename 文件名
     * @param trajectory 输出轨迹
     * @return 是否成功加载
     */
    bool loadTrajectoryFromFile(const std::string& filename, std::vector<TrajectoryPoint>& trajectory);

    /**
     * @brief 保存轨迹到文件
     * @param filename 文件名
     * @param trajectory 轨迹
     * @return 是否成功保存
     */
    bool saveTrajectoryToFile(const std::string& filename, const std::vector<TrajectoryPoint>& trajectory);

    /**
     * @brief 时间对齐轨迹
     * @param traj1 轨迹1
     * @param traj2 轨迹2
     * @param aligned_traj1 对齐后的轨迹1
     * @param aligned_traj2 对齐后的轨迹2
     * @return 是否成功对齐
     */
    bool alignTrajectories(const std::vector<TrajectoryPoint>& traj1,
                          const std::vector<TrajectoryPoint>& traj2,
                          std::vector<TrajectoryPoint>& aligned_traj1,
                          std::vector<TrajectoryPoint>& aligned_traj2);

    /**
     * @brief 计算轨迹长度
     * @param trajectory 轨迹
     * @return 轨迹长度
     */
    double computeTrajectoryLength(const std::vector<TrajectoryPoint>& trajectory);

    /**
     * @brief 计算平均速度
     * @param trajectory 轨迹
     * @return 平均速度
     */
    double computeAverageSpeed(const std::vector<TrajectoryPoint>& trajectory);

    /**
     * @brief 计算两个位姿之间的变换
     * @param pose1 位姿1
     * @param pose2 位姿2
     * @return 变换矩阵
     */
    Eigen::Matrix4d computeTransformation(const TrajectoryPoint& pose1, const TrajectoryPoint& pose2);

    /**
     * @brief 计算变换的平移误差
     * @param transform 变换矩阵
     * @return 平移误差
     */
    double computeTranslationError(const Eigen::Matrix4d& transform);

    /**
     * @brief 计算变换的旋转误差
     * @param transform 变换矩阵
     * @return 旋转误差（弧度）
     */
    double computeRotationError(const Eigen::Matrix4d& transform);

    /**
     * @brief 插值轨迹点
     * @param trajectory 轨迹
     * @param timestamp 目标时间戳
     * @return 插值后的轨迹点
     */
    TrajectoryPoint interpolateTrajectoryPoint(const std::vector<TrajectoryPoint>& trajectory, double timestamp);
};

} 

#endif 
