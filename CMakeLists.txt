cmake_minimum_required (VERSION 3.0)
project (Project)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package (OpenCV 4.2 REQUIRED)


include_directories(${OpenCV_INCLUDE_DIRS})
add_executable (Project Project.cpp)
target_link_libraries (Project
  Eigen3::Eigen
  ${OpenCV_LIBRARIES})
