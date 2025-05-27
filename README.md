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
make
