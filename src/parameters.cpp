#include "parameters.h"

bool is_first_frame = true;
double lidar_end_time = 0.0, first_lidar_time = 0.0, time_con = 0.0;
int pcd_index = 0;
IVoxType::Options ivox_options_;
int ivox_nearby_type = 6;
double max_match_dist = 1.0;

std::vector<double> extrinT(3, 0.0);
std::vector<double> extrinR(9, 0.0);
// state_input state_in;
// state_output state_out;
std::string lid_topic, imu_topic;
bool prop_at_freq_of_imu = true, check_satu = true, con_frame = false, cut_frame = false;
bool use_imu_as_input = false, space_down_sample = true, publish_odometry_without_downsample = false;
int  init_map_size = 10, con_frame_num = 1;
double match_s = 81, satu_acc, satu_gyro, cut_frame_time_interval = 0.1;
float  plane_thr = 0.1f;
double filter_size_surf_min = 0.5, filter_size_map_min = 0.5, fov_deg = 180;
// double cube_len = 2000; 
bool   imu_en = true;
double imu_time_inte = 0.005;
double laser_point_cov = 0.01, acc_norm;
double vel_cov, acc_cov_input, gyr_cov_input;
double gyr_cov_output, acc_cov_output, b_gyr_cov, b_acc_cov;
double imu_meas_acc_cov, imu_meas_omg_cov; 
int    lidar_type, pcd_save_interval;
std::vector<double> gravity_init, gravity;
bool   runtime_pos_log, pcd_save_en, path_en, extrinsic_est_en = true;
bool   scan_pub_en;
shared_ptr<Preprocess> p_pre;
shared_ptr<ImuProcess> p_imu;
double time_diff_lidar_to_imu = 0.0;

int cut_frame_num = 1, orig_odom_freq = 10;
double online_refine_time = 20.0; //unit: s
double time_update_last = 0.0, time_current = 0.0, time_predict_last_const = 0.0, t_last = 0.0;

float vf_scan_res_min;
float vf_scan_res_step;
int vf_max_num;
float vf_scan_res_max;
float process_t_max;
bool asyn_locmap;
bool pure_loc;
std::string map_path;

void readParameters(const YAML::Node &config) {
  // if (config["pure_loc"]) {
  //     pure_loc = config["pure_loc"].as<bool>(false);
  //     std::cout << "pure_loc: " << pure_loc << std::endl;
  // } else {
  //     std::cerr << "Error: pure_loc is not valid!" << std::endl;
  // }
  // if (config["mapping"]["gravity"]) {
  //   if(config["mapping"]["gravity"].IsSequence()){
  //     gravity = config["mapping"]["gravity"].as<std::vector<double>>();
  //     std::cout << "gravity: " << gravity[2] << std::endl;
  //   }else {
  //     std::cerr << "Error: mapping.gravity is not a sequence!" << std::endl;
  //   }
  // } else {
  //     std::cerr << "Error: mapping.gravity is not valid!" << std::endl;
  // }

  p_pre.reset(new Preprocess());
  p_imu.reset(new ImuProcess());
  pure_loc = config["pure_loc"].as<bool>(false);
  map_path = config["map_path"].as<std::string>("0");
  use_imu_as_input = config["use_imu_as_input"].as<bool>(false);
  check_satu = config["check_satu"].as<bool>(true);
  init_map_size = config["init_map_size"].as<int>(100);
  process_t_max = config["process_t_max"].as<float>(1.0);
  asyn_locmap = config["asyn_locmap"].as<bool>(false);

  lid_topic = config["common"]["lid_topic"].as<std::string>(".livox.lidar");
  imu_topic = config["common"]["imu_topic"].as<std::string>(".livox.imu");

  imu_en = config["mapping"]["imu_en"].as<bool>(true);
  b_gyr_cov = config["mapping"]["b_gyr_cov"].as<double>(0.0001);
  b_acc_cov = config["mapping"]["b_acc_cov"].as<double>(0.0001);
  acc_cov_input = config["mapping"]["acc_cov_input"].as<double>(0.1);
  gyr_cov_input = config["mapping"]["gyr_cov_input"].as<double>(0.1);
  extrinsic_est_en = config["mapping"]["extrinsic_est_en"].as<bool>(false);
  satu_acc = config["mapping"]["satu_acc"].as<double>(3.0);
  satu_gyro = config["mapping"]["satu_gyro"].as<double>(35.0);
  acc_norm = config["mapping"]["acc_norm"].as<double>(1.0);
  gyr_cov_output = config["mapping"]["gyr_cov_output"].as<double>(0.1);
  acc_cov_output = config["mapping"]["acc_cov_output"].as<double>(0.1);
  laser_point_cov = config["mapping"]["lidar_meas_cov"].as<double>(0.1);
  imu_meas_acc_cov = config["mapping"]["imu_meas_acc_cov"].as<double>(0.1);
  imu_meas_omg_cov = config["mapping"]["imu_meas_omg_cov"].as<double>(0.1);
  plane_thr = config["mapping"]["plane_thr"].as<float>(0.05f);
  match_s = config["mapping"]["match_s"].as<double>(81);
  max_match_dist = config["mapping"]["max_match_dist"].as<double>(1.0);
  gravity = config["mapping"]["gravity"].as<std::vector<double>>();
  extrinT = config["mapping"]["extrinsic_T"].as<std::vector<double>>();
  extrinR = config["mapping"]["extrinsic_R"].as<std::vector<double>>();

  space_down_sample = config["preprocess"]["space_down_sample"].as<bool>(true);
  vf_scan_res_min = config["preprocess"]["vf_scan_res_min"].as<float>(0.1);
  vf_scan_res_step = config["preprocess"]["vf_scan_res_step"].as<float>(0.1);
  vf_max_num = config["preprocess"]["vf_max_num"].as<int>(500);
  vf_scan_res_max = config["preprocess"]["vf_scan_res_max"].as<float>(1.0);

  p_pre->blind = config["preprocess"]["blind"].as<double>(1.0);
  p_pre->det_range = config["preprocess"]["det_range"].as<double>(1.0);
  p_pre->point_filter_num = config["preprocess"]["point_filter_num"].as<int>(2);
  p_pre->lidar_type = config["preprocess"]["lidar_type"].as<int>(1);

  ivox_options_.resolution_ = config["mapping"]["ivox_grid_resolution"].as<float>(0.2);
  ivox_nearby_type = config["mapping"]["ivox_nearby_type"].as<int>(18);
  ivox_options_.insert_resolution_ = config["mapping"]["filter_size_map"].as<float>(0.1);
  if (ivox_nearby_type == 0) {
    ivox_options_.nearby_type_ = IVoxType::NearbyType::CENTER;
  } else if (ivox_nearby_type == 6) {
    ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY6;
  } else if (ivox_nearby_type == 18) {
    ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
  } else if (ivox_nearby_type == 26) {
    ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY26;
  } else {
    ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
  }
}

void reset_cov_output(Eigen::Matrix<double, 30, 30> & P_init_output)
{
    P_init_output = MD(30, 30)::Identity() * 0.01;
    P_init_output.block<3, 3>(21, 21) = MD(3,3)::Identity() * 0.0001;
    // P_init_output.block<6, 6>(6, 6) = MD(6,6)::Identity() * 0.0001;
    P_init_output.block<6, 6>(24, 24) = MD(6,6)::Identity() * 0.001;
}

void param_reset(){
  time_update_last = 0.0, time_current = 0.0, time_predict_last_const = 0.0, t_last = 0.0;
  Eigen::Matrix<double, 30, 30> P_init_output;
  reset_cov_output(P_init_output);


  p_pre.reset(new Preprocess());
  p_imu.reset(new ImuProcess());
}

