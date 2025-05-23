#pragma once

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <cstring>
#include "preprocess.h"
#include <common_lib.h>
#include "IMU_Processing.h"
#include <mutex>
#include <omp.h>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <ivox/ivox3d.h>
#include <condition_variable>
#include <pcl/common/transforms.h>

#include <yaml-cpp/yaml.h>

using IVoxType = faster_lio::IVox<3, faster_lio::IVoxNodeType::DEFAULT, PointType>;

extern string root_dir;
extern bool is_first_frame;
extern double lidar_end_time, first_lidar_time, time_con;
extern double last_timestamp_lidar, last_timestamp_imu;
extern int pcd_index;
extern IVoxType::Options ivox_options_;
extern int ivox_nearby_type;
extern double max_match_dist;
// extern state_input state_in;
// extern state_output state_out;
extern std::string lid_topic, imu_topic;
extern bool prop_at_freq_of_imu, check_satu, con_frame, cut_frame;
extern bool use_imu_as_input, space_down_sample;
extern bool extrinsic_est_en, publish_odometry_without_downsample;
extern int  init_map_size, con_frame_num;
extern double match_s, satu_acc, satu_gyro, cut_frame_time_interval;
extern float  plane_thr;
extern double filter_size_surf_min, filter_size_map_min, fov_deg;
// extern double cube_len; 
extern bool   imu_en;
extern double imu_time_inte;
extern double laser_point_cov, acc_norm;
extern double acc_cov_input, gyr_cov_input, vel_cov;
extern double gyr_cov_output, acc_cov_output, b_gyr_cov, b_acc_cov;
extern double imu_meas_acc_cov, imu_meas_omg_cov; 
extern int    lidar_type, pcd_save_interval;
extern std::vector<double> gravity_init, gravity;
extern bool   runtime_pos_log, pcd_save_en, path_en;
extern bool   scan_pub_en;
extern shared_ptr<Preprocess> p_pre;
extern shared_ptr<ImuProcess> p_imu;
extern bool is_first_frame;

extern std::vector<double> extrinT;
extern std::vector<double> extrinR;
extern double time_diff_lidar_to_imu;
extern int cut_frame_num, orig_odom_freq;
extern double online_refine_time; //unit: s

extern ofstream fout_out, fout_imu_pbp;
void readParameters(const YAML::Node &config);

void reset_cov_output(Eigen::Matrix<double, 30, 30> & P_init_output);
void param_reset();

extern double time_update_last, time_current, time_predict_last_const, t_last;
extern float vf_scan_res_min;
extern float vf_scan_res_step;
extern int vf_max_num;
extern float vf_scan_res_max;
extern float process_t_max;

extern bool pure_loc;
extern bool asyn_locmap;
extern std::string map_path;