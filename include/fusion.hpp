#ifndef FUSION_HPP
#define FUSION_HPP

#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/opencv.hpp>
struct IMUData {
    double timestamp;
    Eigen::Vector3d acc;
    Eigen::Vector3d gyro;
};

struct Pose {
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
};

class SensorFusion {
public:
    SensorFusion();
    void processIMU(const IMUData& imu);
    void processLiDAR(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    void processCamera(const cv::Mat& image);
    Pose getCurrentPose() const;

private:
    Pose current_pose_;
    Eigen::MatrixXd state_;
    Eigen::MatrixXd cov_;
};

#endif

