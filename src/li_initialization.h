#pragma once

#include <common_lib.h>
// #include "Estimator.h"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include "parameters.h"

class li_initialization {
public:
    std::deque<PointCloudXYZI::Ptr>  lidar_buffer;
    std::deque<double>               time_buffer;
    // std::deque<sensor_msgs::Imu::Ptr> imu_deque;
    std::deque<sensor_msgs::msg::Imu::SharedPtr> imu_deque;
    bool lidar_pushed = false;

    // ros::Publisher pubOdom_imufreq;

    double last_timestamp_lidar = -1.0, last_timestamp_imu = -1.0;

    bool reset_flag = false;

    li_initialization(){};
    ~li_initialization(){};

    // void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr msg); 
    void standard_pcl_cbk(const std::shared_ptr<const sensor_msgs::msg::PointCloud2> msg);
    // void livox_pcl_cbk(const std::shared_ptr<const sensor_msgs::msg::PointCloud2> msg);
    void imu_cbk(const std::shared_ptr<const sensor_msgs::msg::Imu> msg_in);
    void reset_cbk(const std::shared_ptr<const std_msgs::msg::Bool> msg);
    void initialpose_callback(const std::shared_ptr<const geometry_msgs::msg::PoseWithCovarianceStamped> input);

    bool sync_packages(MeasureGroup &meas, bool imu_need_init);
    void reset();
   

  bool use_ManualReloPose_;
  double manual_Relo_Maxscore_;
  double ManualReloPose_z_ = 0.0;
  bool manual_reloc_flag = false;
  Eigen::Matrix4d manual_reloc_pose = Eigen::Matrix4d::Identity();
private:
};
// #endif