cmake_minimum_required(VERSION 2.8.3)
project(simulated_lidar_scanner)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  tf
  tf_conversions
  roslib
  urdf
  interactive_markers
  visualization_msgs
  )

find_package(VTK 6.2 REQUIRED)
include(${VTK_USE_FILE})

find_package(cmake_modules REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES simulated_lidar_scanner
  CATKIN_DEPENDS
    roscpp
  DEPENDS
    VTK
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(synthetic_lidar_scanner SHARED
 src/synthetic_lidar_scanner/vtkLidarScanner.cxx
 src/synthetic_lidar_scanner/vtkRay.cxx
 src/synthetic_lidar_scanner/vtkLidarPoint.cxx
)

target_link_libraries(synthetic_lidar_scanner
 ${catkin_LIBRARIES}
 ${VTK_LIBRARIES}
)

add_executable(simulated_lidar_scanner_node src/lidar_scanner_node.cpp
                                            src/lidar_scanner_simulator.cpp
                                            src/scene_builder.cpp)

target_link_libraries(simulated_lidar_scanner_node
  ${catkin_LIBRARIES}
  ${VTK_LIBRARIES}
  synthetic_lidar_scanner)

add_executable(scanner_relocator src/scanner_relocator.cpp)

target_link_libraries(scanner_relocator
  ${catkin_LIBRARIES})
