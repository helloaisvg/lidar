#include "lidar_odometry/imu_integration.h"
#include <algorithm>
#include <iostream>
#include <cmath>

namespace lidar_odometry {

IMUIntegration::IMUIntegration(const Eigen::Vector3d& gravity, size_t max_buffer_size)
    : position_(Eigen::Vector3d::Zero())
    , velocity_(Eigen::Vector3d::Zero())
    , orientation_(Eigen::Quaterniond::Identity())
    , gravity_(gravity)
    , max_buffer_size_(max_buffer_size)
    , is_initialized_(false)
    , last_timestamp_(0.0) {
}

void IMUIntegration::addIMUData(const IMUData& imu_data) {
    imu_buffer_.push_back(imu_data);
    
    // 保持缓冲区大小
    if (imu_buffer_.size() > max_buffer_size_) {
        imu_buffer_.pop_front();
    }
    
    // 如果是第一个数据，初始化
    if (!is_initialized_) {
        last_timestamp_ = imu_data.timestamp;
        is_initialized_ = true;
        return;
    }
    
    // 执行积分
    double dt = imu_data.timestamp - last_timestamp_;
    if (dt > 0.0 && dt < 1.0) {  // 避免异常时间间隔
        integrateStep(imu_data, dt);
        last_timestamp_ = imu_data.timestamp;
    }
}

Eigen::Matrix4d IMUIntegration::predictPoseTransformation(double start_time, double end_time) {
    if (imu_buffer_.empty()) {
        return Eigen::Matrix4d::Identity();
    }
    
    // 找到时间范围内的IMU数据
    std::vector<IMUData> relevant_data;
    for (const auto& data : imu_buffer_) {
        if (data.timestamp >= start_time && data.timestamp <= end_time) {
            relevant_data.push_back(data);
        }
    }
    
    if (relevant_data.empty()) {
        return Eigen::Matrix4d::Identity();
    }
    
    // 按时间排序
    std::sort(relevant_data.begin(), relevant_data.end(),
              [](const IMUData& a, const IMUData& b) {
                  return a.timestamp < b.timestamp;
              });
    
    // 保存当前状态
    Eigen::Vector3d saved_position = position_;
    Eigen::Vector3d saved_velocity = velocity_;
    Eigen::Quaterniond saved_orientation = orientation_;
    
    // 执行积分
    for (size_t i = 1; i < relevant_data.size(); ++i) {
        double dt = relevant_data[i].timestamp - relevant_data[i-1].timestamp;
        if (dt > 0.0 && dt < 1.0) {
            integrateStep(relevant_data[i], dt);
        }
    }
    
    // 计算变换矩阵
    Eigen::Matrix4d transformation = buildTransformationMatrix(position_, orientation_);
    
    // 恢复状态
    position_ = saved_position;
    velocity_ = saved_velocity;
    orientation_ = saved_orientation;
    
    return transformation;
}

Eigen::Matrix4d IMUIntegration::getCurrentPose() const {
    return buildTransformationMatrix(position_, orientation_);
}

Eigen::Vector3d IMUIntegration::getCurrentVelocity() const {
    return velocity_;
}

Eigen::Quaterniond IMUIntegration::getCurrentOrientation() const {
    return orientation_;
}

void IMUIntegration::reset() {
    position_ = Eigen::Vector3d::Zero();
    velocity_ = Eigen::Vector3d::Zero();
    orientation_ = Eigen::Quaterniond::Identity();
    imu_buffer_.clear();
    is_initialized_ = false;
    last_timestamp_ = 0.0;
}

void IMUIntegration::setInitialPose(const Eigen::Vector3d& position,
                                   const Eigen::Quaterniond& orientation,
                                   const Eigen::Vector3d& velocity) {
    position_ = position;
    orientation_ = orientation;
    velocity_ = velocity;
    is_initialized_ = true;
}

void IMUIntegration::setGravity(const Eigen::Vector3d& gravity) {
    gravity_ = gravity;
}

size_t IMUIntegration::getIMUDataCount() const {
    return imu_buffer_.size();
}

void IMUIntegration::clearOldData(double current_time, double time_window) {
    while (!imu_buffer_.empty() && 
           current_time - imu_buffer_.front().timestamp > time_window) {
        imu_buffer_.pop_front();
    }
}

IMUData IMUIntegration::interpolateIMUData(double t) const {
    if (imu_buffer_.empty()) {
        return IMUData();
    }
    
    if (imu_buffer_.size() == 1) {
        return imu_buffer_.front();
    }
    
    // 找到时间t前后的数据
    auto it = std::lower_bound(imu_buffer_.begin(), imu_buffer_.end(), t,
                               [](const IMUData& data, double time) {
                                   return data.timestamp < time;
                               });
    
    if (it == imu_buffer_.begin()) {
        return imu_buffer_.front();
    }
    
    if (it == imu_buffer_.end()) {
        return imu_buffer_.back();
    }
    
    // 线性插值
    const IMUData& data1 = *(it - 1);
    const IMUData& data2 = *it;
    
    double alpha = (t - data1.timestamp) / (data2.timestamp - data1.timestamp);
    alpha = std::max(0.0, std::min(1.0, alpha));
    
    IMUData interpolated_data;
    interpolated_data.timestamp = t;
    interpolated_data.linear_acceleration = (1.0 - alpha) * data1.linear_acceleration + 
                                           alpha * data2.linear_acceleration;
    interpolated_data.angular_velocity = (1.0 - alpha) * data1.angular_velocity + 
                                        alpha * data2.angular_velocity;
    
    return interpolated_data;
}

void IMUIntegration::integrateStep(const IMUData& imu_data, double dt) {
    // 更新姿态（使用角速度积分）
    Eigen::Vector3d rotation_vector = imu_data.angular_velocity * dt;
    Eigen::Matrix3d rotation_matrix = rotationVectorToMatrix(rotation_vector);
    Eigen::Quaterniond delta_orientation(rotation_matrix);
    orientation_ = orientation_ * delta_orientation;
    orientation_.normalize();
    
    // 将加速度从IMU坐标系转换到世界坐标系
    Eigen::Vector3d world_acceleration = orientation_ * imu_data.linear_acceleration;
    
    // 去除重力影响
    world_acceleration -= gravity_;
    
    // 更新位置和速度（使用梯形积分）
    velocity_ += world_acceleration * dt;
    position_ += velocity_ * dt + 0.5 * world_acceleration * dt * dt;
}

Eigen::Matrix3d IMUIntegration::rotationVectorToMatrix(const Eigen::Vector3d& rotation_vector) const {
    double angle = rotation_vector.norm();
    if (angle < 1e-8) {
        return Eigen::Matrix3d::Identity();
    }
    
    Eigen::Vector3d axis = rotation_vector / angle;
    Eigen::AngleAxisd rotation(angle, axis);
    return rotation.toRotationMatrix();
}

Eigen::Vector3d IMUIntegration::rotationMatrixToVector(const Eigen::Matrix3d& rotation_matrix) const {
    Eigen::AngleAxisd rotation(rotation_matrix);
    return rotation.angle() * rotation.axis();
}

Eigen::Matrix4d IMUIntegration::buildTransformationMatrix(const Eigen::Vector3d& position,
                                                        const Eigen::Quaterniond& orientation) const {
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    transformation.block<3, 3>(0, 0) = orientation.toRotationMatrix();
    transformation.block<3, 1>(0, 3) = position;
    return transformation;
}

} 
