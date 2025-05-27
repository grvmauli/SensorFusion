#include <iostream>
#include <fstream>
#include "fusion.hpp"
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>



int main() {
    SensorFusion fusion;

    // 🔧 Create a more powerful viewer (persistent outside the loop)
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Fusion Viewer"));
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    // 🧭 Store trajectory
    std::vector<Eigen::Vector3d> path;

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

        // 🖼️ Replace cloud each frame
        if (!viewer->updatePointCloud(cloud, "cloud"))
            viewer->addPointCloud(cloud, "cloud");

        // Simulate IMU
        IMUData imu;
        imu.timestamp = i * 0.01;
        imu.acc = Eigen::Vector3d(0.1, 0, 0);
        imu.gyro = Eigen::Vector3d(0, 0, 0.01);
        fusion.processIMU(imu);

        auto pose = fusion.getCurrentPose();
        path.push_back(pose.position);

        // Draw trajectory
	if (path.size() >= 2) {
	    size_t last = path.size() - 1;

	    // Get previous and current Eigen::Vector3d points
	    Eigen::Vector3d prev_point = path[last - 1];
	    Eigen::Vector3d curr_point = path[last];

	    // Create pcl points with float casting
	    pcl::PointXYZ pt1(
		static_cast<float>(prev_point.x()),
		static_cast<float>(prev_point.y()),
		static_cast<float>(prev_point.z())
	    );

	    pcl::PointXYZ pt2(
		static_cast<float>(curr_point.x()),
		static_cast<float>(curr_point.y()),
		static_cast<float>(curr_point.z())
	    );

	    std::string line_id = "line_" + std::to_string(last);
	    viewer->addLine(pt1, pt2, 1.0, 0.0, 0.0, line_id);
	}



        std::cout << "Pose: " << pose.position.transpose() << std::endl;

        viewer->spinOnce(10);
    }

    // Keep viewer open after loop
    while (!viewer->wasStopped()) {
        viewer->spinOnce(10);
    }

    return 0;
}
