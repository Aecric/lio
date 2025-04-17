#include "preprocess.h"


Preprocess::Preprocess()
  :lidar_type(AVIA), blind(0.01), point_filter_num(1), det_range(1000)
{
  // inf_bound = 10;
  // N_SCANS   = 6;
  // SCAN_RATE = 10;
  // group_size = 8;
  // disA = 0.01;
  // disA = 0.1; // B?
  // p2l_ratio = 225;
  // limit_maxmid =6.25;
  // limit_midmin =6.25;
  // limit_maxmin = 3.24;
  // jump_up_limit = 170.0;
  // jump_down_limit = 8.0;
  // cos160 = 160.0;
  // edgea = 2;
  // edgeb = 0.1;
  // smallp_intersect = 172.5;
  // smallp_ratio = 1.2;
  // given_offset_time = false;

  // jump_up_limit = cos(jump_up_limit/180*M_PI);
  // jump_down_limit = cos(jump_down_limit/180*M_PI);
  // cos160 = cos(cos160/180*M_PI);
  // smallp_intersect = cos(smallp_intersect/180*M_PI);
}

Preprocess::~Preprocess() {}

void Preprocess::process(const std::shared_ptr<const sensor_msgs::msg::PointCloud2> msg, PointCloudXYZI::Ptr &pcl_out)
{
  // switch (time_unit)
  // {
  //   case SEC:
  //     time_unit_scale = 1.e3f;
  //     break;
  //   case MS:
  //     time_unit_scale = 1.f;
  //     break;
  //   case US:
  //     time_unit_scale = 1.e-3f;
  //     break;
  //   case NS:
  //     time_unit_scale = 1.e-6f;
  //     break;
  //   default:
  //     time_unit_scale = 1.f;
  //     break;
  // }

  // switch (lidar_type)
  // {
  // case AVIA:
  //   avia_handler(msg);
  // case OUST64:
  //   oust64_handler(msg);
  //   break;

  // case VELO16:
  //   velodyne_handler(msg);
  //   break;
  
  // case HESAIxt32:
  //   hesai_handler(msg);
  //   break;
  
  // default:
  //   printf("Error LiDAR Type");
  //   break;
  // }

  avia_handler(msg);
  *pcl_out = pl_surf;
}


void Preprocess::avia_handler(const std::shared_ptr<const sensor_msgs::msg::PointCloud2> msg){

  pcl::PointCloud<PointXYZRTLT> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  double time_sec = rclcpp::Time(msg->header.stamp).seconds();

  int plsize = pl_orig.size();
  pl_surf.clear();
  pl_surf.reserve(plsize);
  
  int valid_num = 0;
  for(uint i=1; i<plsize; i++)
  {
    // 0x30 的二进制表示是 00110000
    // 0x30 的二进制表示是 00010000
    // 关于tag的判断用于筛选回波序号，只使用第0、第一个回波
    if((pl_orig.points[i].tag & 0x30) == 0x10 || (pl_orig.points[i].tag & 0x30) == 0x00)
    {
      valid_num ++;
      if (valid_num % point_filter_num == 0) // 根据point_filter_num的余数跳点
      {
        PointType pl_;
        pl_.x = pl_orig.points[i].x;
        pl_.y = pl_orig.points[i].y;
        pl_.z = pl_orig.points[i].z;
        pl_.intensity = pl_orig.points[i].intensity; // z; //
        pl_.curvature = float(pl_orig.points[i].timestamp - time_sec*1000000000)/float(1000000); // use curvature as time of each laser points, curvature unit: ms
        
        // 剔除太近的点
        if(abs(pl_.x) < blind && abs(pl_.y) < blind && abs(pl_.z) < blind){
          continue;
        }
        pl_surf.push_back(pl_);
      }
    }
  }
}



PointCloudXYZI Preprocess::down_sample(bool _down_sample, float _scan_res_min, float _max_num, float _scan_res_max,
                                     float _scan_res_step, const PointCloudXYZI::Ptr &_feats_undistort){
    PointCloudXYZI::Ptr _feats_down_body(new PointCloudXYZI());

    if(_down_sample){    
        if(_feats_undistort->size() < _max_num){
            *_feats_down_body = *_feats_undistort;
            sort(_feats_down_body->points.begin(), _feats_down_body->points.end(), time_list); 
            // std::cout << "vf_scan_res_: " << 0.0 << std::endl;
        }else{
          // downSizeFilterSurf.setInputCloud(_feats_undistort);
          // downSizeFilterSurf.filter(*feats_down_body);

          PointCloudXYZI frame_dowsampled;
          frame_dowsampled.points.reserve(_feats_undistort->points.size());
          tsl::robin_map<Voxel, int, VoxelHash> grid;
          grid.reserve(_feats_undistort->points.size());
          float vf_scan_res_ = _scan_res_min;
          // float 
          while (true)
          {
              for (int i = 0; i < _feats_undistort->points.size(); i++)
              {
                  Eigen::Vector3f point(_feats_undistort->points[i].x, _feats_undistort->points[i].y, _feats_undistort->points[i].z);
                  Voxel voxel = Voxel((point / vf_scan_res_).cast<int>());
                  if (grid.contains(voxel))
                  {
                  continue;
                  }
                  else
                  {
                  grid.insert({voxel, i});
                  }
                  if (grid.size() > _max_num)
                  {
                  break;
                  }
              }

              // 分辨率太大
              if (vf_scan_res_ > _scan_res_max)
              {
                  break;
              }

              if (grid.size() > _max_num){
                  vf_scan_res_ += _scan_res_step;
                  grid.clear();
              }else
              {
                  break;
              }
          }
          // std::cout << "grid size: " << grid.size() << std::endl;
          // std::cout << "voxel_size: " << voxel_size << std::endl;
          for (const auto &[voxel, index] : grid)
          {
              (void)voxel;
              frame_dowsampled.points.emplace_back(_feats_undistort->points[index]);
          }

          *_feats_down_body = frame_dowsampled;

          sort(_feats_down_body->points.begin(), _feats_down_body->points.end(), time_list); 
          // std::cout << "vf_scan_res_: " << vf_scan_res_ << std::endl;
        }

    }else{
        *_feats_down_body = *_feats_undistort;
        sort(_feats_down_body->points.begin(), _feats_down_body->points.end(), time_list); 
    }
    return *_feats_down_body;
}


// void Preprocess::set(bool feat_en, int lid_type, double bld, int pfilt_num)
// {
//   lidar_type = lid_type;
//   blind = bld;
//   point_filter_num = pfilt_num;
// }


// void Preprocess::process(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out)
// {  
//   avia_handler(msg);
//   *pcl_out = pl_surf;
// }

// void Preprocess::avia_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg)
// {
//   pl_surf.clear();
//   pl_corn.clear();
//   pl_full.clear();
//   double t1 = omp_get_wtime();
//   int plsize = msg->point_num;

//   pl_corn.reserve(plsize);
//   pl_surf.reserve(plsize);
//   pl_full.resize(plsize);

//   // for(int i=0; i<N_SCANS; i++)
//   // {
//   //   pl_buff[i].clear();
//   //   pl_buff[i].reserve(plsize);
//   // }
//   // uint valid_num = 0;
  
  
//   for(uint i=1; i<plsize; i++)
//   {
//     // 0x30 的二进制表示是 00110000
//     // 0x30 的二进制表示是 00010000
//     // 关于tag的判断用于筛选回波序号，只使用第0、第一个回波
//     if((msg->points[i].line < N_SCANS) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
//     {
//       // valid_num ++;
//       // if (valid_num % point_filter_num == 0) // 根据point_filter_num的余数跳点
//       // {
//         pl_full[i].x = msg->points[i].x;
//         pl_full[i].y = msg->points[i].y;
//         pl_full[i].z = msg->points[i].z;
//         pl_full[i].intensity = msg->points[i].reflectivity; // z; //
//         pl_full[i].curvature = msg->points[i].offset_time / float(1000000); // use curvature as time of each laser points, curvature unit: ms
        
//         // 剔除太近的点
//         // double dist = pl_full[i].x * pl_full[i].x + pl_full[i].y * pl_full[i].y + pl_full[i].z * pl_full[i].z;
//         // if (dist < blind * blind || dist > det_range * det_range) continue;
//         if(abs(pl_full[i].x) < blind && abs(pl_full[i].y) < blind && abs(pl_full[i].z) < blind){
//           continue;
//         }


//         // 剔除重合点
//         if(((abs(pl_full[i].x - pl_full[i-1].x) > 1e-7) 
//             || (abs(pl_full[i].y - pl_full[i-1].y) > 1e-7)
//             || (abs(pl_full[i].z - pl_full[i-1].z) > 1e-7)))
//         {
//           pl_surf.push_back(pl_full[i]);
//         }
//       // }
//     }
//   }

// }

// void Preprocess::oust64_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
// {
//   pl_surf.clear();
//   pl_corn.clear();
//   pl_full.clear();
//   pcl::PointCloud<ouster_ros::Point> pl_orig;
//   pcl::fromROSMsg(*msg, pl_orig);
//   int plsize = pl_orig.size();
//   pl_corn.reserve(plsize);
//   pl_surf.reserve(plsize);
  
  
//   double time_stamp = msg->header.stamp.toSec();
//   // cout << "===================================" << endl;
//   // printf("Pt size = %d, N_SCANS = %d\r\n", plsize, N_SCANS);
//   for (int i = 0; i < pl_orig.points.size(); i++)
//   {
//     if (i % point_filter_num != 0) continue;

//     double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y + pl_orig.points[i].z * pl_orig.points[i].z;
    
//     if (range < (blind * blind) || range > det_range * det_range || isnan(pl_orig.points[i].x) || isnan(pl_orig.points[i].y) || isnan(pl_orig.points[i].z)) continue;
    
//     Eigen::Vector3d pt_vec;
//     PointType added_pt;
//     added_pt.x = pl_orig.points[i].x;
//     added_pt.y = pl_orig.points[i].y;
//     added_pt.z = pl_orig.points[i].z;
//     added_pt.intensity = pl_orig.points[i].intensity;
//     added_pt.normal_x = 0;
//     added_pt.normal_y = 0;
//     added_pt.normal_z = 0;
//     added_pt.curvature = pl_orig.points[i].t * time_unit_scale; // curvature unit: ms

//     pl_surf.points.push_back(added_pt);
//   }
  
//   // pub_func(pl_surf, pub_full, msg->header.stamp);
//   // pub_func(pl_surf, pub_corn, msg->header.stamp);
// }

// void Preprocess::velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
// {
//     pl_surf.clear();
//     pl_corn.clear();
//     pl_full.clear();

//     pcl::PointCloud<velodyne_ros::Point> pl_orig;
//     pcl::fromROSMsg(*msg, pl_orig);
//     int plsize = pl_orig.points.size();
//     if (plsize == 0) return;
//     pl_surf.reserve(plsize);
    
//     /*** These variables only works when no point timestamps given ***/
//     double omega_l = 0.361 * SCAN_RATE;       // scan angular velocity
//     std::vector<bool> is_first(N_SCANS,true);
//     std::vector<double> yaw_fp(N_SCANS, 0.0);      // yaw of first scan point
//     std::vector<float> yaw_last(N_SCANS, 0.0);   // yaw of last scan point
//     std::vector<float> time_last(N_SCANS, 0.0);  // last offset time
//     /*****************************************************************/

//     if (pl_orig.points[plsize - 1].time > 0)
//     {
//       given_offset_time = true;
//     }
//     else
//     {
//       given_offset_time = false;
//       // double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578;
//       // double yaw_end  = yaw_first;
//       // int layer_first = pl_orig.points[0].ring;
//       // for (uint i = plsize - 1; i > 0; i--)
//       // {
//       //   if (pl_orig.points[i].ring == layer_first)
//       //   {
//       //     yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;
//       //     break;
//       //   }
//       // }
//     }

//     for (int i = 0; i < plsize; i++)
//     {
//       PointType added_pt;
//       // cout<<"!!!!!!"<<i<<" "<<plsize<<endl;
      
//       added_pt.normal_x = 0;
//       added_pt.normal_y = 0;
//       added_pt.normal_z = 0;
//       added_pt.x = pl_orig.points[i].x;
//       added_pt.y = pl_orig.points[i].y;
//       added_pt.z = pl_orig.points[i].z;
//       added_pt.intensity = pl_orig.points[i].intensity;
//       added_pt.curvature = pl_orig.points[i].time * time_unit_scale;  // curvature unit: ms // cout<<added_pt.curvature<<endl;
//       if (i % point_filter_num != 0 || std::isnan(added_pt.x) || std::isnan(added_pt.y) || std::isnan(added_pt.z)) continue;

//       if (!given_offset_time)
//       {
//         // int unit_size = plsize / 16;
//         // int layer = i / unit_size; // pl_orig.points[i].ring;
//         // cout << "check layer:" << unit_size << ";" << i << ";" << layer << endl;
//         int layer = 0;
//         double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

//         if (is_first[layer])
//         {
//           // printf("layer: %d; is first: %d", layer, is_first[layer]);
//             yaw_fp[layer]=yaw_angle;
//             is_first[layer]=false;
//             added_pt.curvature = 0.0;
//             yaw_last[layer]=yaw_angle;
//             time_last[layer]=added_pt.curvature;
//             continue;
//         }

//         // compute offset time
//         if (yaw_angle < yaw_fp[layer])
//         {
//           added_pt.curvature = (yaw_fp[layer]-yaw_angle) / omega_l;
//           // added_pt.curvature = (yaw_angle + 360.0 - yaw_fp[layer]) / omega_l;
//         }
//         else
//         {
//           added_pt.curvature = (yaw_fp[layer]-yaw_angle+360.0) / omega_l;
//           // added_pt.curvature = (yaw_angle-yaw_fp[layer]) / omega_l;
//         }

//         // if (added_pt.curvature < time_last[layer])  added_pt.curvature+=360.0/omega_l;

//         // yaw_last[layer] = yaw_angle;
//         // time_last[layer]=added_pt.curvature;
//       }

//       // if (i % point_filter_num == 0 && !std::isnan(added_pt.x) && !std::isnan(added_pt.y) && !std::isnan(added_pt.z))
//       double dist = added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z;
//       {
//         if(dist > (blind * blind)
//         && dist < (det_range * det_range))
//         {
//           pl_surf.points.push_back(added_pt);
//         }
//       }
//     }
    
// }

// void Preprocess::hesai_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
// {
//     pl_surf.clear();
//     pl_corn.clear();
//     pl_full.clear();

//     pcl::PointCloud<hesai_ros::Point> pl_orig;
//     pcl::fromROSMsg(*msg, pl_orig);
//     int plsize = pl_orig.points.size();
//     if (plsize == 0) return;
//     pl_surf.reserve(plsize);
    
//     /*** These variables only works when no point timestamps given ***/
//     double omega_l = 0.361 * SCAN_RATE;       // scan angular velocity
//     std::vector<bool> is_first(N_SCANS,true);
//     std::vector<double> yaw_fp(N_SCANS, 0.0);      // yaw of first scan point
//     std::vector<float> yaw_last(N_SCANS, 0.0);   // yaw of last scan point
//     std::vector<float> time_last(N_SCANS, 0.0);  // last offset time
//     /*****************************************************************/

//     if (pl_orig.points[plsize - 1].timestamp > 0)
//     {
//       given_offset_time = true;
//     }
//     else
//     {
//       given_offset_time = false;
//       double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578;
//       double yaw_end  = yaw_first;
//       int layer_first = pl_orig.points[0].ring;
//       for (uint i = plsize - 1; i > 0; i--)
//       {
//         if (pl_orig.points[i].ring == layer_first)
//         {
//           yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;
//           break;
//         }
//       }
//     }

//     double time_head = pl_orig.points[0].timestamp;
    
//     for (int i = 0; i < plsize; i++)
//     {
//       PointType added_pt;
//       // cout<<"!!!!!!"<<i<<" "<<plsize<<endl;
      
//       added_pt.normal_x = 0;
//       added_pt.normal_y = 0;
//       added_pt.normal_z = 0;
//       added_pt.x = pl_orig.points[i].x;
//       added_pt.y = pl_orig.points[i].y;
//       added_pt.z = pl_orig.points[i].z;
//       added_pt.intensity = pl_orig.points[i].intensity;
//       added_pt.curvature = (pl_orig.points[i].timestamp - time_head) * time_unit_scale;  // curvature unit: ms // cout<<added_pt.curvature<<endl;
//       if (!given_offset_time)
//       {
//         int layer = pl_orig.points[i].ring;
//         double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

//         if (is_first[layer])
//         {
//           // printf("layer: %d; is first: %d", layer, is_first[layer]);
//             yaw_fp[layer]=yaw_angle;
//             is_first[layer]=false;
//             added_pt.curvature = 0.0;
//             yaw_last[layer]=yaw_angle;
//             time_last[layer]=added_pt.curvature;
//             continue;
//         }

//         // compute offset time
//         if (yaw_angle <= yaw_fp[layer])
//         {
//           added_pt.curvature = (yaw_fp[layer]-yaw_angle) / omega_l;
//         }
//         else
//         {
//           added_pt.curvature = (yaw_fp[layer]-yaw_angle+360.0) / omega_l;
//         }

//         if (added_pt.curvature < time_last[layer])  added_pt.curvature+=360.0/omega_l;

//         yaw_last[layer] = yaw_angle;
//         time_last[layer]=added_pt.curvature;
//       }

//       if (i % point_filter_num == 0 && !std::isnan(added_pt.x) && !std::isnan(added_pt.y) && !std::isnan(added_pt.z))
//       {
//         if(added_pt.x*added_pt.x+added_pt.y*added_pt.y+added_pt.z*added_pt.z > (blind * blind)
//         &&added_pt.x*added_pt.x+added_pt.y*added_pt.y+added_pt.z*added_pt.z < (det_range * det_range))
//         {
//           pl_surf.points.push_back(added_pt);
//         }
//       }
//     }
    
// }

// void Preprocess::give_feature(pcl::PointCloud<PointType> &pl, vector<orgtype> &types)
// {
//   int plsize = pl.size();
//   int plsize2;
//   if(plsize == 0)
//   {
//     printf("something wrong\n");
//     return;
//   }
//   uint head = 0;

//   while(types[head].range < blind)
//   {
//     head++;
//   }

//   // Surf
//   plsize2 = (plsize > group_size) ? (plsize - group_size) : 0;

//   Eigen::Vector3d curr_direct(Eigen::Vector3d::Zero());
//   Eigen::Vector3d last_direct(Eigen::Vector3d::Zero());

//   uint i_nex = 0, i2;
//   uint last_i = 0; uint last_i_nex = 0;
//   int last_state = 0;
//   int plane_type;

//   for(uint i=head; i<plsize2; i++)
//   {
//     if(types[i].range < blind)
//     {
//       continue;
//     }

//     i2 = i;

//     plane_type = plane_judge(pl, types, i, i_nex, curr_direct);
    
//     if(plane_type == 1)
//     {
//       for(uint j=i; j<=i_nex; j++)
//       { 
//         if(j!=i && j!=i_nex)
//         {
//           types[j].ftype = Real_Plane;
//         }
//         else
//         {
//           types[j].ftype = Poss_Plane;
//         }
//       }
      
//       // if(last_state==1 && fabs(last_direct.sum())>0.5)
//       if(last_state==1 && last_direct.norm()>0.1)
//       {
//         double mod = last_direct.transpose() * curr_direct;
//         if(mod>-0.707 && mod<0.707)
//         {
//           types[i].ftype = Edge_Plane;
//         }
//         else
//         {
//           types[i].ftype = Real_Plane;
//         }
//       }
      
//       i = i_nex - 1;
//       last_state = 1;
//     }
//     else // if(plane_type == 2)
//     {
//       i = i_nex;
//       last_state = 0;
//     }

//     last_i = i2;
//     last_i_nex = i_nex;
//     last_direct = curr_direct;
//   }

//   plsize2 = plsize > 3 ? plsize - 3 : 0;
//   for(uint i=head+3; i<plsize2; i++)
//   {
//     if(types[i].range<blind || types[i].ftype>=Real_Plane)
//     {
//       continue;
//     }

//     if(types[i-1].dista<1e-16 || types[i].dista<1e-16)
//     {
//       continue;
//     }

//     Eigen::Vector3d vec_a(pl[i].x, pl[i].y, pl[i].z);
//     Eigen::Vector3d vecs[2];

//     for(int j=0; j<2; j++)
//     {
//       int m = -1;
//       if(j == 1)
//       {
//         m = 1;
//       }

//       if(types[i+m].range < blind)
//       {
//         if(types[i].range > inf_bound)
//         {
//           types[i].edj[j] = Nr_inf;
//         }
//         else
//         {
//           types[i].edj[j] = Nr_blind;
//         }
//         continue;
//       }

//       vecs[j] = Eigen::Vector3d(pl[i+m].x, pl[i+m].y, pl[i+m].z);
//       vecs[j] = vecs[j] - vec_a;
      
//       types[i].angle[j] = vec_a.dot(vecs[j]) / vec_a.norm() / vecs[j].norm();
//       if(types[i].angle[j] < jump_up_limit)
//       {
//         types[i].edj[j] = Nr_180;
//       }
//       else if(types[i].angle[j] > jump_down_limit)
//       {
//         types[i].edj[j] = Nr_zero;
//       }
//     }

//     types[i].intersect = vecs[Prev].dot(vecs[Next]) / vecs[Prev].norm() / vecs[Next].norm();
//     if(types[i].edj[Prev]==Nr_nor && types[i].edj[Next]==Nr_zero && types[i].dista>0.0225 && types[i].dista>4*types[i-1].dista)
//     {
//       if(types[i].intersect > cos160)
//       {
//         if(edge_jump_judge(pl, types, i, Prev))
//         {
//           types[i].ftype = Edge_Jump;
//         }
//       }
//     }
//     else if(types[i].edj[Prev]==Nr_zero && types[i].edj[Next]== Nr_nor && types[i-1].dista>0.0225 && types[i-1].dista>4*types[i].dista)
//     {
//       if(types[i].intersect > cos160)
//       {
//         if(edge_jump_judge(pl, types, i, Next))
//         {
//           types[i].ftype = Edge_Jump;
//         }
//       }
//     }
//     else if(types[i].edj[Prev]==Nr_nor && types[i].edj[Next]==Nr_inf)
//     {
//       if(edge_jump_judge(pl, types, i, Prev))
//       {
//         types[i].ftype = Edge_Jump;
//       }
//     }
//     else if(types[i].edj[Prev]==Nr_inf && types[i].edj[Next]==Nr_nor)
//     {
//       if(edge_jump_judge(pl, types, i, Next))
//       {
//         types[i].ftype = Edge_Jump;
//       }
     
//     }
//     else if(types[i].edj[Prev]>Nr_nor && types[i].edj[Next]>Nr_nor)
//     {
//       if(types[i].ftype == Nor)
//       {
//         types[i].ftype = Wire;
//       }
//     }
//   }

//   plsize2 = plsize-1;
//   double ratio;
//   for(uint i=head+1; i<plsize2; i++)
//   {
//     if(types[i].range<blind || types[i-1].range<blind || types[i+1].range<blind)
//     {
//       continue;
//     }
    
//     if(types[i-1].dista<1e-8 || types[i].dista<1e-8)
//     {
//       continue;
//     }

//     if(types[i].ftype == Nor)
//     {
//       if(types[i-1].dista > types[i].dista)
//       {
//         ratio = types[i-1].dista / types[i].dista;
//       }
//       else
//       {
//         ratio = types[i].dista / types[i-1].dista;
//       }

//       if(types[i].intersect<smallp_intersect && ratio < smallp_ratio)
//       {
//         if(types[i-1].ftype == Nor)
//         {
//           types[i-1].ftype = Real_Plane;
//         }
//         if(types[i+1].ftype == Nor)
//         {
//           types[i+1].ftype = Real_Plane;
//         }
//         types[i].ftype = Real_Plane;
//       }
//     }
//   }

//   int last_surface = -1;
//   for(uint j=head; j<plsize; j++)
//   {
//     if(types[j].ftype==Poss_Plane || types[j].ftype==Real_Plane)
//     {
//       if(last_surface == -1)
//       {
//         last_surface = j;
//       }
    
//       if(j == uint(last_surface+point_filter_num-1))
//       {
//         PointType ap;
//         ap.x = pl[j].x;
//         ap.y = pl[j].y;
//         ap.z = pl[j].z;
//         ap.intensity = pl[j].intensity;
//         ap.curvature = pl[j].curvature;
//         pl_surf.push_back(ap);

//         last_surface = -1;
//       }
//     }
//     else
//     {
//       if(types[j].ftype==Edge_Jump || types[j].ftype==Edge_Plane)
//       {
//         pl_corn.push_back(pl[j]);
//       }
//       if(last_surface != -1)
//       {
//         PointType ap;
//         for(uint k=last_surface; k<j; k++)
//         {
//           ap.x += pl[k].x;
//           ap.y += pl[k].y;
//           ap.z += pl[k].z;
//           ap.intensity += pl[k].intensity;
//           ap.curvature += pl[k].curvature;
//         }
//         ap.x /= (j-last_surface);
//         ap.y /= (j-last_surface);
//         ap.z /= (j-last_surface);
//         ap.intensity /= (j-last_surface);
//         ap.curvature /= (j-last_surface);
//         pl_surf.push_back(ap);
//       }
//       last_surface = -1;
//     }
//   }
// }

// void Preprocess::pub_func(PointCloudXYZI &pl, const ros::Time &ct)
// {
//   pl.height = 1; pl.width = pl.size();
//   sensor_msgs::PointCloud2 output;
//   pcl::toROSMsg(pl, output);
//   output.header.frame_id = "livox";
//   output.header.stamp = ct;
// }

// int Preprocess::plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct)
// {
//   double group_dis = disA*types[i_cur].range + disB;
//   group_dis = group_dis * group_dis;
//   // i_nex = i_cur;

//   double two_dis;
//   vector<double> disarr;
//   disarr.reserve(20);

//   for(i_nex=i_cur; i_nex<i_cur+group_size; i_nex++)
//   {
//     if(types[i_nex].range < blind)
//     {
//       curr_direct.setZero();
//       return 2;
//     }
//     disarr.push_back(types[i_nex].dista);
//   }
  
//   for(;;)
//   {
//     if((i_cur >= pl.size()) || (i_nex >= pl.size())) break;

//     if(types[i_nex].range < blind)
//     {
//       curr_direct.setZero();
//       return 2;
//     }
//     vx = pl[i_nex].x - pl[i_cur].x;
//     vy = pl[i_nex].y - pl[i_cur].y;
//     vz = pl[i_nex].z - pl[i_cur].z;
//     two_dis = vx*vx + vy*vy + vz*vz;
//     if(two_dis >= group_dis)
//     {
//       break;
//     }
//     disarr.push_back(types[i_nex].dista);
//     i_nex++;
//   }

//   double leng_wid = 0;
//   double v1[3], v2[3];
//   for(uint j=i_cur+1; j<i_nex; j++)
//   {
//     if((j >= pl.size()) || (i_cur >= pl.size())) break;
//     v1[0] = pl[j].x - pl[i_cur].x;
//     v1[1] = pl[j].y - pl[i_cur].y;
//     v1[2] = pl[j].z - pl[i_cur].z;

//     v2[0] = v1[1]*vz - vy*v1[2];
//     v2[1] = v1[2]*vx - v1[0]*vz;
//     v2[2] = v1[0]*vy - vx*v1[1];

//     double lw = v2[0]*v2[0] + v2[1]*v2[1] + v2[2]*v2[2];
//     if(lw > leng_wid)
//     {
//       leng_wid = lw;
//     }
//   }


//   if((two_dis*two_dis/leng_wid) < p2l_ratio)
//   {
//     curr_direct.setZero();
//     return 0;
//   }

//   uint disarrsize = disarr.size();
//   for(uint j=0; j<disarrsize-1; j++)
//   {
//     for(uint k=j+1; k<disarrsize; k++)
//     {
//       if(disarr[j] < disarr[k])
//       {
//         leng_wid = disarr[j];
//         disarr[j] = disarr[k];
//         disarr[k] = leng_wid;
//       }
//     }
//   }

//   if(disarr[disarr.size()-2] < 1e-16)
//   {
//     curr_direct.setZero();
//     return 0;
//   }

//   if(lidar_type==AVIA)
//   {
//     double dismax_mid = disarr[0]/disarr[disarrsize/2];
//     double dismid_min = disarr[disarrsize/2]/disarr[disarrsize-2];

//     if(dismax_mid>=limit_maxmid || dismid_min>=limit_midmin)
//     {
//       curr_direct.setZero();
//       return 0;
//     }
//   }
//   else
//   {
//     double dismax_min = disarr[0] / disarr[disarrsize-2];
//     if(dismax_min >= limit_maxmin)
//     {
//       curr_direct.setZero();
//       return 0;
//     }
//   }
  
//   curr_direct << vx, vy, vz;
//   curr_direct.normalize();
//   return 1;
// }

// bool Preprocess::edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir)
// {
//   if(nor_dir == 0)
//   {
//     if(types[i-1].range<blind || types[i-2].range<blind)
//     {
//       return false;
//     }
//   }
//   else if(nor_dir == 1)
//   {
//     if(types[i+1].range<blind || types[i+2].range<blind)
//     {
//       return false;
//     }
//   }
//   double d1 = types[i+nor_dir-1].dista;
//   double d2 = types[i+3*nor_dir-2].dista;
//   double d;

//   if(d1<d2)
//   {
//     d = d1;
//     d1 = d2;
//     d2 = d;
//   }

//   d1 = sqrt(d1);
//   d2 = sqrt(d2);

 
//   if(d1>edgea*d2 || (d1-d2)>edgeb)
//   {
//     return false;
//   }
  
//   return true;
// }
