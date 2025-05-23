cmake_minimum_required(VERSION 3.3)
project(lio)

SET(CMAKE_BUILD_TYPE "Release") # Release Debug

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -fexceptions")

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)


# for ros2
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(pointlio_mapping 
  src/main.cpp
  src/laserMapping.cpp 
  src/li_initialization.cpp 
  src/parameters.cpp 
  src/preprocess.cpp 
  src/Estimator.cpp 
  src/IMU_Processing.cpp 
  # src/MapProcess.cpp 
  src/LocalMapProcess.cpp
)
target_link_libraries(pointlio_mapping 
  ${PCL_LIBRARIES} 
  ${nav_msgs_LIBRARIES} 
  ${pcl_conversions_LIBRARIES} 
  tf2_geometry_msgs::tf2_geometry_msgs
  Eigen3::Eigen
  yaml-cpp
  pthread
)


target_include_directories(
  pointlio_mapping
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
         ${nav_msgs_INCLUDE_DIRS}
         ${pcl_conversions_INCLUDE_DIRS})

# include_directories(
# 	${catkin_INCLUDE_DIRS} 
#   ${EIGEN3_INCLUDE_DIR}
#   ${PCL_INCLUDE_DIRS}
#   ${PYTHON_INCLUDE_DIRS}
#   include)

# link_directories(
# 	include
# 	${PCL_LIBRARY_DIRS}
# )



# add_executable(pointlio_mapping src/laserMapping.cpp 
#                 src/li_initialization.cpp src/parameters.cpp src/preprocess.cpp src/Estimator.cpp 
#                 src/IMU_Processing.cpp src/MapProcess.cpp src/LocalMapProcess.cpp)
# target_link_libraries(pointlio_mapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES} yaml-cpp)
# target_include_directories(pointlio_mapping PRIVATE ${PYTHON_INCLUDE_DIRS})

install(TARGETS
        pointlio_mapping
        DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)


ament_export_dependencies(rclcpp  geometry_msgs nav_msgs sensor_msgs std_msgs  pcl_conversions tf2_ros tf2 tf2_geometry_msgs  Eigen3 PCL)

ament_package()




