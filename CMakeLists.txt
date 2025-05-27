cmake_minimum_required(VERSION 3.10)
project(SensorFusion)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
include_directories(/home/grv/Sensorfusion/include)
add_executable(sensor_fusion main.cpp src/fusion.cpp)
target_link_libraries(sensor_fusion ${PCL_LIBRARIES} ${OpenCV_LIBS})

