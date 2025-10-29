#ifndef IMU_INTEGRATION_H
#define IMU_INTEGRATION_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <deque>
#include <chrono>
#include <memory>

namespace lidar_odometry {

/**
 * @brief IMU数据结构
 */
struct IMUData {
    double timestamp;                    // 时间戳
    Eigen::Vector3d linear_acceleration; // 线性加速度 (m/s^2)
    Eigen::Vector3d angular_velocity;    // 角速度 (rad/s)
    Eigen::Vector3d magnetic_field;      // 磁场强度 (可选)
    
    IMUData() : timestamp(0.0) {}
    
    IMUData(double t, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro)
        : timestamp(t), linear_acceleration(acc), angular_velocity(gyro) {}
};

/**
 * @brief IMU积分器类
 * 实现IMU数据的积分以预测位姿变换
 */
class IMUIntegration {
public:
    /**
     * @brief 构造函数
     * @param gravity 重力加速度
     * @param max_buffer_size 最大缓冲区大小
     */
    IMUIntegration(const Eigen::Vector3d& gravity = Eigen::Vector3d(0, 0, -9.81),
                   size_t max_buffer_size = 1000);

    /**
     * @brief 析构函数
     */
    ~IMUIntegration() = default;

    /**
     * @brief 添加IMU数据
     * @param imu_data IMU数据
     */
    void addIMUData(const IMUData& imu_data);

    /**
     * @brief 预测位姿变换
     * @param start_time 开始时间
     * @param end_time 结束时间
     * @return 预测的位姿变换矩阵
     */
    Eigen::Matrix4d predictPoseTransformation(double start_time, double end_time);

    /**
     * @brief 获取当前位姿
     * @return 当前位姿变换矩阵
     */
    Eigen::Matrix4d getCurrentPose() const;

    /**
     * @brief 获取当前速度
     * @return 当前速度向量
     */
    Eigen::Vector3d getCurrentVelocity() const;

    /**
     * @brief 获取当前姿态
     * @return 当前姿态四元数
     */
    Eigen::Quaterniond getCurrentOrientation() const;

    /**
     * @brief 重置积分器
     */
    void reset();

    /**
     * @brief 设置初始位姿
     * @param position 初始位置
     * @param orientation 初始姿态
     * @param velocity 初始速度
     */
    void setInitialPose(const Eigen::Vector3d& position,
                       const Eigen::Quaterniond& orientation,
                       const Eigen::Vector3d& velocity = Eigen::Vector3d::Zero());

    /**
     * @brief 设置重力向量
     * @param gravity 重力向量
     */
    void setGravity(const Eigen::Vector3d& gravity);

    /**
     * @brief 获取IMU数据数量
     * @return IMU数据数量
     */
    size_t getIMUDataCount() const;

    /**
     * @brief 清除旧数据
     * @param current_time 当前时间
     * @param time_window 时间窗口
     */
    void clearOldData(double current_time, double time_window = 10.0);

private:
    // 积分状态
    Eigen::Vector3d position_;           // 当前位置
    Eigen::Vector3d velocity_;           // 当前速度
    Eigen::Quaterniond orientation_;    // 当前姿态
    Eigen::Vector3d gravity_;            // 重力向量
    
    // IMU数据缓冲区
    std::deque<IMUData> imu_buffer_;     // IMU数据缓冲区
    size_t max_buffer_size_;              // 最大缓冲区大小
    
    // 积分参数
    bool is_initialized_;                 // 是否已初始化
    double last_timestamp_;               // 最后时间戳
    
    /**
     * @brief 线性插值IMU数据
     * @param t 目标时间
     * @return 插值后的IMU数据
     */
    IMUData interpolateIMUData(double t) const;

    /**
     * @brief 执行一步积分
     * @param imu_data IMU数据
     * @param dt 时间步长
     */
    void integrateStep(const IMUData& imu_data, double dt);

    /**
     * @brief 旋转向量到旋转矩阵
     * @param rotation_vector 旋转向量
     * @return 旋转矩阵
     */
    Eigen::Matrix3d rotationVectorToMatrix(const Eigen::Vector3d& rotation_vector) const;

    /**
     * @brief 旋转矩阵到旋转向量
     * @param rotation_matrix 旋转矩阵
     * @return 旋转向量
     */
    Eigen::Vector3d rotationMatrixToVector(const Eigen::Matrix3d& rotation_matrix) const;

    /**
     * @brief 构建变换矩阵
     * @param position 位置
     * @param orientation 姿态
     * @return 变换矩阵
     */
    Eigen::Matrix4d buildTransformationMatrix(const Eigen::Vector3d& position,
                                              const Eigen::Quaterniond& orientation) const;
};

} 

#endif 
