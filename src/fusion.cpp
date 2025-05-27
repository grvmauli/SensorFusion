#include "fusion.hpp"
#include <iostream>

SensorFusion::SensorFusion() {
    state_ = Eigen::MatrixXd::Zero(15, 1); // Pose + Vel + Bias
    cov_ = Eigen::MatrixXd::Identity(15, 15) * 0.01;
    current_pose_.position = Eigen::Vector3d::Zero();
    current_pose_.orientation = Eigen::Quaterniond::Identity();
}

void SensorFusion::processIMU(const IMUData& imu) {
    // Basic integration (placeholder)
    current_pose_.position += imu.acc * 0.01; // Very simplified
}

void SensorFusion::processLiDAR(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    // ICP or scan-matching (not implemented here)
}

void SensorFusion::processCamera(const cv::Mat& image) {
    // Feature tracking / visual odometry (not implemented here)
}

Pose SensorFusion::getCurrentPose() const {
    return current_pose_;
}

