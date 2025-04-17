#pragma once
#include <cmath>
#include <math.h>
// #include <deque>
// #include <mutex>
// #include <thread>
#include <csignal>
// #include <ros/ros.h>
// #include <so3_math.h>
#include <Eigen/Eigen>
// #include "Estimator.h"
#include <common_lib.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <sensor_msgs/PointCloud2.h>

/// *************Preconfiguration

#define MAX_INI_COUNT (100)

/// *************IMU Process and undistortion
class ImuProcess
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcess();
  ~ImuProcess();
  
  void Reset();
  void Process(const MeasureGroup &meas);
  void set_gyr_cov(const V3D &scaler);
  void set_acc_cov(const V3D &scaler);
  void Set_init(Eigen::Vector3d &tmp_gravity, Eigen::Matrix3d &rot);

  MD(12, 12) state_cov = MD(12, 12)::Identity();
  int    lidar_type;
  V3D    gravity_;
  bool   imu_en;
  V3D    mean_acc;
  bool   imu_need_init_ = true;
  bool   gravity_align_ = false;
  bool   b_first_frame_ = true;
  double time_last_scan = 0.0;
  V3D cov_gyr_scale = V3D(0.0001, 0.0001, 0.0001);
  V3D cov_vel_scale = V3D(0.0001, 0.0001, 0.0001);

  int init_move_count = 0;
  double G_m_s2 = 9.81;

 private:
  void IMU_init(const MeasureGroup &meas, int &N);
  V3D mean_gyr;
  int init_iter_num = 1;
};
