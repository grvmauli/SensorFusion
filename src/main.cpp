#include <iostream>
#include <fstream>
#include "fusion.hpp"
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

int main() {
    SensorFusion fusion;
    pcl::visualization::CloudViewer viewer("LiDAR Viewer");

    for (int i = 0; i < 100; ++i) {
        // Load image
        std::string img_path = "data/image/" + std::to_string(i) + ".png";
        cv::Mat image = cv::imread(img_path, cv::IMREAD_COLOR);
        if (image.empty()) continue;

        fusion.processCamera(image);
        cv::imshow("Camera", image);
        cv::waitKey(1);

        // Load LiDAR
        std::string pcd_path = "data/lidar/" + std::to_string(i) + ".pcd";
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *cloud) == -1) continue;

        fusion.processLiDAR(cloud);
        viewer.showCloud(cloud);

        // Simulate IMU
        IMUData imu;
        imu.timestamp = i * 0.01;
        imu.acc = Eigen::Vector3d(0.1, 0, 0); // Fake data
        imu.gyro = Eigen::Vector3d(0, 0, 0.01);
        fusion.processIMU(imu);

        auto pose = fusion.getCurrentPose();
        std::cout << "Pose: " << pose.position.transpose() << std::endl;
    }

    return 0;
}

