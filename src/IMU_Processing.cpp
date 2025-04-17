#include "IMU_Processing.h"


void ImuProcess::set_gyr_cov(const V3D &scaler)
{
  cov_gyr_scale = scaler;
}

void ImuProcess::set_acc_cov(const V3D &scaler)
{
  cov_vel_scale = scaler;
}

ImuProcess::ImuProcess()
    : b_first_frame_(true), imu_need_init_(true)
{
  imu_en = true;
  init_iter_num = 1;
  mean_acc      = V3D(0, 0, 0.0);
  mean_gyr      = V3D(0, 0, 0);
  gravity_align_ = false;
  state_cov.setIdentity();
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset() 
{
  mean_acc      = V3D(0, 0, 0.0);
  mean_gyr      = V3D(0, 0, 0);
  imu_need_init_    = true;
  init_iter_num     = 1;
  gravity_align_   = false;
  b_first_frame_   = true;
  init_move_count = 0;

  
  time_last_scan = 0.0;
}

void ImuProcess::Set_init(Eigen::Vector3d &tmp_gravity, Eigen::Matrix3d &rot)
{
  // /** 1. initializing the gravity, gyro bias, acc and gyro covariance
  //  ** 2. normalize the acceleration measurenments to unit gravity **/
  // // V3D tmp_gravity = - mean_acc / mean_acc.norm() * G_m_s2; // state_gravity;
  // M3D hat_grav;
  // // gravity_在上一步p_imu->gravity_ << VEC_FROM_ARRAY(gravity);通过参数得到
  // // hat_grav 反对称矩阵，用于描述叉乘运算

  // /*
  //   重力与加速度计的旋转矩阵计算
  //   利用叉乘运算、点乘运算得到轴角变换，然后罗德里格公式得到旋转矩阵
  //   对于两个向量u、v，有 a=uxv/|uxv|（利用叉乘，转轴），cosθ=u^T·v/|u|*|v|（利用点乘，转角）， R=exp(axθ)（罗德里格公式）
  //   v=R⋅u
  //   所以这里求得的旋转为 从重力坐标系 到 当前acc坐标系 的坐标系变换，即 Rag
  // */
  // // 叉乘反对称矩阵，这里实际在前面加了一个负号，因为 gravity_ 代表重力大小方向是负的，而重力的imu测量应该正过来
  // hat_grav << 0.0, gravity_(2), -gravity_(1),
  //             -gravity_(2), 0.0, gravity_(0),
  //             gravity_(1), -gravity_(0), 0.0;
  // // 转轴
  // double align_norm = (hat_grav * tmp_gravity).norm() / tmp_gravity.norm() / gravity_.norm();
  // // 转角，这里是否需要加负号？我给改了加上负号
  // double align_cos = -gravity_.transpose() * tmp_gravity;
  // align_cos = align_cos / gravity_.norm() / tmp_gravity.norm();

  // if (align_norm < 1e-6)
  // {
  //   if (align_cos > 1e-6)
  //   {
  //     // 如果两个向量接近正交（align_norm < 1e-6），且方向相同（align_cos > 1e-6），则设定旋转矩阵为单位矩阵 Eye3d，表示无需旋转。
  //     rot = Eye3d;
  //   }
  //   else
  //   {
  //     // 如果方向相反（align_cos 小于 1e-6），则旋转矩阵设为负的单位矩阵 -Eye3d，表示需要180度旋转。
  //     rot = -Eye3d;
  //   }
  // }
  // else
  // {
  //   // 如果两个向量既不正交也不平行，通过 align_angle 计算旋转轴与角度，再通过 Exp 函数（李代数的指数映射）将旋转轴角转换为旋转矩阵 rot。
  //   V3D align_angle = hat_grav * tmp_gravity / (hat_grav * tmp_gravity).norm() * acos(align_cos); 
  //   rot = Exp(align_angle(0), align_angle(1), align_angle(2));
  // }
  // yaw分量的偏差： 当两个向量的夹角非常接近 180 度时，可能会引入额外的 yaw
  // 分量。
  // 即使加速度计的数据与重力方向非常接近，实际计算时可能会存在微小的偏差，导致
  // yaw 角度出现不必要的变化。 旋转轴的不稳定性：
  // 旋转轴本身也可能会受到微小误差的影响，导致原本应该是绕 x 或 y
  // 轴的旋转，被误判为绕 z 轴的旋转，从而导致 yaw 分量的出现。 当检测到夹角接近
  // 180 度（如误差小于 5
  // 度）时，可以直接将旋转矩阵设为单位矩阵或处理为仅有俯仰和翻滚的旋转，而不计算
  // yaw 方向的旋转。 目前测试，在翻转接近180度时，yaw角会存在不必要的值：

  V3D norm_gravity = tmp_gravity / tmp_gravity.norm();
  float pitch = -asin(norm_gravity[0]);
  float roll = atan2(norm_gravity[1], norm_gravity[2]);

  Eigen::Matrix3d Rga = Eigen::Matrix3d::Identity();
  {
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(0.0, Eigen::Vector3d::UnitZ());
    Rga = yawAngle * pitchAngle * rollAngle;
  }
  rot = Rga.transpose();

  // std::cout << "pitch: " << pitch * 180 / 3.14 << std::endl;
  // std::cout << "roll: " << roll * 180 / 3.14 << std::endl;
}

void ImuProcess::IMU_init(const MeasureGroup &meas, int &N)
{
  /** 1. initializing the gravity, gyro bias, acc and gyro covariance
   ** 2. normalize the acceleration measurenments to unit gravity **/
  printf("IMU Initializing: %.1f \n", double(N) / MAX_INI_COUNT * 100); // 这玩意而的N是上一次的
  V3D cur_acc, cur_gyr;
  
  // 第一帧的时候直接取第一个数据给到mean_acc、mean_gyr，感觉是没有必要的操作，主要是针对无imu情况吧
  if (b_first_frame_)
  {
    Reset();
    const auto &imu_acc = meas.imu.front()->linear_acceleration;
    const auto &gyr_acc = meas.imu.front()->angular_velocity;
    mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

    // float acc_scare = (mean_acc).norm() > 5 ? (mean_acc).norm() : ((mean_acc).norm() * G_m_s2);
    // if(acc_scare > 5 && mean_gyr.norm() < 0.1 ){
      b_first_frame_ = false;
    // }else{
    //   b_first_frame_ = true;
    //   return;
    // }
  }

  // 取平均得到mean_acc、mean_gyr
  for (const auto &imu : meas.imu)
  {
    const auto &imu_acc = imu->linear_acceleration;
    const auto &gyr_acc = imu->angular_velocity;
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

    // mean_acc      += (cur_acc - mean_acc) / N;
    // mean_gyr      += (cur_gyr - mean_gyr) / N;
    // float acc_scare = (mean_acc/N - cur_acc).norm() > 5 ? (mean_acc/N - cur_acc).norm() : ((mean_acc/N - cur_acc).norm() * G_m_s2);
    // if(acc_scare < 0.3 && cur_gyr.norm() < 0.1 ){
      mean_acc      += cur_acc;
      mean_gyr      += cur_gyr;
      N ++;
    // }else{
    //   init_move_count++;
    // }
  }

  // if(init_move_count > 10){
  //   std::cout << "(mean_acc/N - cur_acc).norm(): " << (mean_acc/N - cur_acc).norm() << std::endl;
  //   std::cout << "cur_gyr.norm(): " << cur_gyr.norm() << std::endl;
  //   std::cout << "-------- IMU INIT MOVE --------\n\n\n" << std::endl;
  //   Reset();
  //   // exit(0);
  // }
}

void ImuProcess::Process(const MeasureGroup &meas)
{  

  // 等待imu数据，其实没啥用，sync_packages已经保证了存在imu数据了
  if(meas.imu.empty())  return;

  // imu_need_init_在p_imu实例化时确定为 true，代表需要imu初始化。 在这里之后会变为 false
  if (imu_need_init_)
  {
    /// The very first lidar frame
    IMU_init(meas, init_iter_num);

    imu_need_init_ = true;

    if (init_iter_num > MAX_INI_COUNT)
    {
      printf("IMU Initializing: %.1f \n", 100.0);
      mean_acc = mean_acc / init_iter_num;
      mean_gyr = mean_gyr / init_iter_num;

      std::cout << "mean_acc: " << mean_acc << std::endl;
      std::cout << "mean_gyr: " << mean_gyr << std::endl;
      std::cout << "-------- IMU INIT --------\n\n\n" << std::endl;


      imu_need_init_ = false;
    }
    return;
  }
  return;
}