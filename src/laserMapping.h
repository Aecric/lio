#pragma once
#include <cmath>
#include <math.h>
#include <iostream>
#include <Eigen/Eigen>
#include <string>
#include <unistd.h>

#include <Eigen/Eigen>

// ROS2 
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "parameters.h"
#include "Estimator.h"
#include "LocalMapProcess.h"
#include "li_initialization.h"

class laserMapping
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  laserMapping(rclcpp::Node::SharedPtr node_);
  ~laserMapping() = default;

  void run();

  void reset_judge();

  bool gravity_init();

  bool map_init();

  LocalMapProcess *localmap_process_handle;
  void setLocalMapHandle(LocalMapProcess *_handle)
  {
    localmap_process_handle = _handle;
  };

  PointCloudXYZI::Ptr init_feats_world;
  MeasureGroup Measures;

  bool init_map = false, flg_first_scan = true;
  // Time Log Variables
  double match_time = 0, solve_time = 0, propag_time = 0;
 private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc;
  // rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr
  //     sub_pcl_livox;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_reset;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_reloc;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserFrameWorld;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserReloc;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer;

  rclcpp::Node::SharedPtr nh;  // ROS2 节点

  li_initialization *li_int;

  template<typename T>
  void set_posestamp(T & out)
  {
      out.position.x = kf_output.x_.pos(0);
      out.position.y = kf_output.x_.pos(1);
      out.position.z = kf_output.x_.pos(2);
      Eigen::Quaterniond q(kf_output.x_.rot);
      out.orientation.x = q.coeffs()[0];
      out.orientation.y = q.coeffs()[1];
      out.orientation.z = q.coeffs()[2];
      out.orientation.w = q.coeffs()[3];
  }

  void publish_odometry()
  {
      nav_msgs::msg::Odometry odom_msg;

      odom_msg.header.frame_id = "camera_init";
      odom_msg.child_frame_id = "body";

      odom_msg.header.stamp = rclcpp::Time(lidar_end_time * 1e9);

      set_posestamp(odom_msg.pose.pose);

      pubOdomAftMapped->publish(odom_msg);

      geometry_msgs::msg::TransformStamped transformStamped;
      transformStamped.header.stamp = odom_msg.header.stamp;
      transformStamped.header.frame_id = "camera_init";
      transformStamped.child_frame_id = "body";
      transformStamped.transform.translation.x = odom_msg.pose.pose.position.x;
      transformStamped.transform.translation.y = odom_msg.pose.pose.position.y;
      transformStamped.transform.translation.z = odom_msg.pose.pose.position.z;
      transformStamped.transform.rotation = odom_msg.pose.pose.orientation;

      tf_broadcaster_->sendTransform(transformStamped);
  }

  void publish_frame_world()
  {
      // 假设 feats_down_body 和 feats_down_world 是已填充好的 PointCloudXYZI::Ptr
      PointCloudXYZI::Ptr laserCloudFullRes = feats_down_body;
      int size = laserCloudFullRes->points.size();

      PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

      for (int i = 0; i < size; ++i)
      {
          laserCloudWorld->points[i].x = feats_down_world->points[i].x;
          laserCloudWorld->points[i].y = feats_down_world->points[i].y;
          laserCloudWorld->points[i].z = feats_down_world->points[i].z;
          laserCloudWorld->points[i].intensity = feats_down_world->points[i].intensity;
      }

      sensor_msgs::msg::PointCloud2 laserCloudMsg;
      pcl::toROSMsg(*laserCloudWorld, laserCloudMsg);

      // 时间戳转换为 ROS2 类型（单位纳秒）
      laserCloudMsg.header.stamp = rclcpp::Time(lidar_end_time * 1e9);
      laserCloudMsg.header.frame_id = "camera_init";

      pubLaserFrameWorld->publish(laserCloudMsg);
  }

};
