# Sensor Fusion with KITTI Dataset

This project implements a basic sensor fusion pipeline using camera, LiDAR, and IMU data from the [KITTI dataset](http://www.cvlibs.net/datasets/kitti/). It visualizes synchronized data in 3D, overlays trajectories, and demonstrates how different sensors can be combined for environment perception.

## 📦 Features

- Load and visualize camera images, LiDAR point clouds, and IMU data
- Convert KITTI Velodyne `.bin` files to `.pcd`
- Extract and parse IMU data
- Render camera frames and point clouds
- Draw estimated trajectory in a 3D PCL viewer

## 🛠 Build Instructions

```bash
mkdir build
cd build
cmake ..

./sensor_fusion
📁 Directory Structure
main.cpp: Entry point for visualization and data loading

include/fusion.hpp / src/fusion.cpp: Fusion logic

scripts/velodyne_bin_to_pcd.py: Convert KITTI LiDAR .bin to .pcd

scripts/extract_imu.py: Extract IMU readings from KITTI oxts folder

data/: Folder for sample images, .pcd files, and IMU text

📚 Requirements
C++17

PCL

OpenCV

Eigen

Python 3 (for data conversion scripts)
make
