#include "laserMapping.h"


double serch_time = 0.0;
using namespace std;     
string root_dir = ROOT_DIR;

laserMapping::laserMapping(rclcpp::Node::SharedPtr node_){

    nh = node_;
    // li_initialization li_int;
    li_int = new li_initialization();

    sub_pcl_pc = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
        lid_topic, rclcpp::SensorDataQoS(),
        std::bind(&li_initialization::standard_pcl_cbk, li_int, std::placeholders::_1));
    sub_imu = nh->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, rclcpp::SensorDataQoS(), std::bind(&li_initialization::imu_cbk, li_int, std::placeholders::_1));
    sub_reset = nh->create_subscription<std_msgs::msg::Bool>(
        "/reset", 1, std::bind(&li_initialization::reset_cbk, li_int, std::placeholders::_1));
    sub_reloc = nh->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 1, std::bind(&li_initialization::initialpose_callback, li_int, std::placeholders::_1));

    // 创建发布者
    pubLaserFrameWorld = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_frame_world", 10);
    pubOdomAftMapped = nh->create_publisher<nav_msgs::msg::Odometry>("/lio_odom", 10);
    pubLaserReloc = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_reloc", 10);
    pubLaserCloudMap = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map", 10);

    //   li_int->pubOdom_imufreq = nh->create_publisher<nav_msgs::msg::Odometry>("/Odom_imufreq", 10);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(nh);
}


void laserMapping::reset_judge()
{
    // 状态复位操作
    if (li_int->reset_flag)
    {
        // 获取局部地图
        if(!pure_loc && ivox_->NumValidGrids() > 1){
            std::vector<PointType> points_all = ivox_->GetAllPoints();
            std::cout << "points_all.size(): " << points_all.size() << std::endl;
            pcl::PointCloud<pcl::PointXYZINormal> clouds_local;
            // 批量转换 std::vector 到 pcl::PointCloud
            clouds_local.points.insert(clouds_local.points.end(), points_all.begin(), points_all.end());

            sensor_msgs::msg::PointCloud2 laserCloudmsg;
            pcl::toROSMsg(clouds_local, laserCloudmsg);
            laserCloudmsg.header.stamp = rclcpp::Time(static_cast<uint64_t>(lidar_end_time * 1e9));
            laserCloudmsg.header.frame_id = "camera_init";
            pubLaserCloudMap->publish(laserCloudmsg);
            

            std::cout << "save map to root_dir: " << root_dir << std::endl;
            pcl::io::savePCDFileBinary(root_dir + "map/map.pcd", clouds_local);
        }

        std::cout << "--------------- reset slam system ---------------\n\n\n\n\n" << std::endl;


        param_reset();
        YAML::Node config = YAML::LoadFile("/home/pf/LJJ_SRC/lio/point_lio/ws_grid/Point-LIO-point-lio-with-grid-map/config/avia.yaml");
        readParameters(config);
        
        p_imu->Reset();    

        Eigen::Matrix<double, 30, 30> P_init_output;
        reset_cov_output(P_init_output);
        kf_output.change_P(P_init_output);
            
        flg_first_scan = true;
        is_first_frame = true;
        init_map = false;

        
        init_feats_world = PointCloudXYZI::Ptr (new PointCloudXYZI());

        Measures = MeasureGroup();


        est_reset();

        li_int->reset();
        li_int->reset_flag = false;

        if(asyn_locmap){
            localmap_process_handle->Reset();
        }
    }
}


bool laserMapping::gravity_init(){
    // 主要是针对初 imu 始化，得到 imu 初始均值，这个会运行几次，MAX_INI_COUNT 参数代表需要的 imu 数据量
    // 完成后才会使 p_imu->imu_need_init_ = false;
    // 在 imu_need_init_ 的时候，会让此次循环退出，以收集更多的imu数据
    // 确保初始化的时候静止1-2s
    if (p_imu->imu_need_init_){
        p_imu->Process(Measures);
        return false;
    }
    // std::cout << "lidar_buffer.size(): " << li_int->lidar_buffer.size() << std::endl;
    // std::cout << "imu_deque.size(): " << li_int->imu_deque.size() << std::endl;


    // 确定重力大小和初始方向
    // 并设置卡尔曼滤波状态变量的方向相关状态
    // p_imu->gravity_align_ 在初始化时为 false ，代表初始化状态、且 imu 数据充足的情况
    // 注意 state_out 只在这里起作用，目的是总结数据给 kf_output.change_x(state_out);
    if (!p_imu->gravity_align_)
    {
        state_output state_out;
        // 注意单位是g还是m/s^2，这里是把p_imu->mean_acc 单位是 g、state_out.gravity单位是m/s^2
        state_out.gravity = -1 * p_imu->mean_acc * G_m_s2 / acc_norm; 
        state_out.acc = p_imu->mean_acc * G_m_s2 / acc_norm;

        // 确保初始化的时候静止1-2s
        {
            // 根据当前加速度计数据 state_out.acc 和重力向量 p_imu->gravity_ ，计算 Rag = rot_init
            Eigen::Matrix3d rot_init;
            // p_imu->gravity_ 只在这个时候有用
            p_imu->gravity_ << VEC_FROM_ARRAY(gravity);
            p_imu->Set_init(state_out.acc, rot_init); // 得到 rot_init 为 Rag
            std::cout << "rot_init(Rga): \n " << rot_init.transpose() << std::endl;
            Eigen::Vector3d euler_xyz = rot_init.transpose().eulerAngles(0, 1, 2);
            std::cout << "INIT Euler angles (XYZ order)(Rag):\n" 
                    << "Roll (X): " << euler_xyz[0]*180/3.14 << " deg\n" 
                    << "Pitch (Y): " << euler_xyz[1]*180/3.14 << " deg\n" 
                    << "Yaw (Z): " << euler_xyz[2]*180/3.14 << " deg\n";

            // 更新初始状态量
            state_out.gravity = p_imu->gravity_;
            state_out.rot = rot_init.transpose();
            
            // 根据旋转矩阵得到当前加速度数据（而且应该不用transpose吧），这里因为重力是向下的与加速度计测量相反，所以前面加负号
            // 多此一举？
            state_out.acc = rot_init * (-state_out.gravity);
        }
        kf_output.change_x(state_out);
        p_imu->gravity_align_ = true;

        // std::cout << "state_out.rot: \n" << state_out.rot << std::endl;
        // std::cout << "state_out.gravity: " << state_out.gravity << std::endl;
        // std::cout << "state_out.acc: " << state_out.acc << std::endl;

        // std::cout << "kf_output.x_.rot: \n" << kf_output.x_.rot << std::endl;
        // std::cout << "kf_output.x_.acc: " << kf_output.x_.acc << std::endl;
        // std::cout << "kf_output.x_.gravity: " << kf_output.x_.gravity << std::endl;
    }
    return true;
}


bool laserMapping::map_init(){
    /*** initialize the map ***/
    // 确保初始化的时候静止1-2s
    if(!init_map)
    {
        feats_down_world->resize(Measures.lidar->size());            
        // 把点转到世界坐标系（重力对齐）
        for(int i = 0; i < Measures.lidar->size(); i++)
        {
            pointBodyToWorld(&(Measures.lidar->points[i]), &(feats_down_world->points[i]));
        }
        // 存入点到init_feats_world
        for (size_t i = 0; i < feats_down_world->size(); i++) 
        {
            init_feats_world->points.emplace_back(feats_down_world->points[i]);
        }
        // 如果点数不够则继续往地图里加点
        if(init_feats_world->size() < init_map_size){
            init_map = false;
        }else{   
            init_map = true;
        }
        // init_map流程可能会存有部分lidar数据缓存，把历史数据删除，重新对齐到最新帧处理
        // imu数据不做删除，会在is_first_frame时自己对齐
        if(init_map){
            if(asyn_locmap){
                localmap_process_handle->set_ivox(*ivox_);
            }else{
                ivox_->AddPoints(init_feats_world->points);
            }

            init_feats_world.reset(new PointCloudXYZI());
            // // delay test
            // sleep(5);
            // std::cout << "lidar_buffer.size(): " << li_int->lidar_buffer.size() << std::endl;
            // std::cout << "imu_deque.size(): " << li_int->imu_deque.size() << std::endl;
            // 清空 deque
            if (!li_int->lidar_buffer.empty()) {
                li_int->lidar_buffer.clear();
                li_int->imu_deque.clear();
                li_int->time_buffer.clear();
            }

            std::cout << "-------- ivox_ map inited --------\n\n\n" << std::endl;
        }
        return false;
    }
    return true;
}


void laserMapping::run()
{
    /*** variables definition and load Parameters***/
    Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
    G_m_s2 = std::sqrt(gravity[0] * gravity[0] + gravity[1] * gravity[1] + gravity[2] * gravity[2]);
    
    p_imu->lidar_type = p_pre->lidar_type;
    p_imu->imu_en = imu_en;
    p_imu->G_m_s2 = G_m_s2;
    
    ivox_ = std::make_shared<IVoxType>(ivox_options_);
    init_feats_world = PointCloudXYZI::Ptr (new PointCloudXYZI());

    cout<<"lidar_type: "<<p_pre->lidar_type<<endl;

    /* 
    初始化卡尔曼滤波器相关，此卡尔曼滤波为误差状态卡尔曼滤波 ESKF（ESEKF） 形式
    get_f_output: 名义状态递推得到预测状态 vel、omg、acc，对应论文的 f(x,w) = [v, w, R.a+g, 0(3*1), 0(3*1), 0(3*1), 0(3*1), 0(3*1)]，PVR之外的其它状态保持不变
                    kf_output.predict(dt, Q_output, input_in, true, false);时被调用 f(x_, i_in)
    df_dx_output: 误差状态转移，根据误差状态的雅可比矩阵，得到误差状态预测
                    kf_output.predict(dt, Q_output, input_in, false, true);时被调用 f_x(x_, i_in)
    h_model_output: 雷达观测，主要观测位姿，其残差是根据当前定位的3D点坐标和地图点坐标的差值，，H矩阵参照论文。satu_check保证在没有点面匹配时不做观测
                    kf_output.update_iterated_dyn_share_IMU();时被调用 h_dyn_share_modified_2(x_, dyn_share)
    h_model_IMU_output: imu原始数据观测，其残差比较直接（z = angvel_avr - s.omg）上一帧数据与当前帧数据的差值，H矩阵参照论文。satu_check保证在加速度计不正常时不做观测
                    kf_output.update_iterated_dyn_share_IMU();时被调用 h_dyn_share_modified_1(x_, dyn_share);
    */
    kf_output.init_dyn_share_modified_3h(get_f_output, df_dx_output, h_model_output, h_model_IMU_output);
    Eigen::Matrix<double, 30, 30> P_init_output;
    reset_cov_output(P_init_output);
    kf_output.change_P(P_init_output);
    Eigen::Matrix<double, 30, 30> Q_output = process_noise_cov_output();

    while(true){
        reset_judge();

        // Measures为输出，输入为lid_topic和imu_topic消息回调得到的数据队列
        // 而其实后面处理，除了初始化的时候用Measures的imu数据，其它情况下都是直接操作imu数据的队列......
        // 且，Measures的imu数据也没有更新过......
        if(!li_int->sync_packages(Measures, p_imu->imu_need_init_)) {
            usleep(2000);
            continue;
        }

        double clc_timestamp_start;{
            struct timespec time1 = {0};
            clock_gettime(CLOCK_REALTIME, &time1);
            clc_timestamp_start = time1.tv_sec + time1.tv_nsec*1e-9;
        }

        if(!gravity_init()){
            // std::cout << "gravity_init() not finished" << std::endl;
            continue;
        }

        if(!map_init()){
            // std::cout << "map_init() not finished" << std::endl;
            continue;
        }

        // // debug
        // std::cout << "lidar_buffer.size(): " << li_int->lidar_buffer.size() << std::endl;
        // std::cout << "imu_deque.size(): " << li_int->imu_deque.size() << std::endl;
        // printf("lidar_end_time: %.4f\n", lidar_end_time);
        // printf("rclcpp::Time(li_int->imu_deque.front()->header.stamp).seconds(): %.4f\n", 
        //     rclcpp::Time(li_int->imu_deque.front()->header.stamp).seconds());
        // printf("rclcpp::Time(li_int->imu_deque.back()->header.stamp).seconds(): %.4f\n", 
        //     rclcpp::Time(li_int->imu_deque.back()->header.stamp).seconds());


///////////////////////////////////////////////////////////////////////////////
///// 初始化完成
///////////////////////////////////////////////////////////////////////////////
        /*** downsample the feature points in a scan ***/
        // 对当前帧雷达数据进行降采样，体素滤波，得到 feats_down_body ，后续处理都针对 feats_down_body
        // 默认分辨率5cm
        // 并按时间戳排序
        *feats_down_body = p_pre->down_sample(true, vf_scan_res_min, vf_max_num, vf_scan_res_max,
                                    vf_scan_res_step, Measures.lidar); 

        // 这一步很重要，对输入的点云数据进行时间压缩处理，基于点的 curvature 属性，将点云序列压缩成一系列的时间段。
        // time_seq 的维度代表若干个时间段内的点云数量（一般为1）
        // time_seq 内所存的元素为点云的索引
        time_seq = time_compressing<int>(feats_down_body); 
        int feats_down_size = feats_down_body->points.size();
        std::cout << "Measures.lidar->size(): " << Measures.lidar->size() << std::endl;
        std::cout << "feats_down_size: " << feats_down_size << std::endl;

        // prepare data pipline
        // 这里就是初始化了三维点云的大小
        // 调用 resize() 后不会清除已有的数据，但会根据新大小对容器进行扩展或缩减
        // 但是在点云观测步骤 update_iterated_dyn_share_modified -> h_model_output 里，每个相关值都会被更新，与feats_down_body一一对应
        feats_down_world->clear();
        feats_down_world->resize(feats_down_size); 
        Nearest_Points.clear();
        Nearest_Points.resize(feats_down_size);
        crossmat_list.clear();
        crossmat_list.resize(feats_down_size);
        pbody_list.clear();
        pbody_list.resize(feats_down_size);
        pabcd_pre.clear();
        pabcd_pre.resize(feats_down_size);
        p_norm_pre.clear();
        p_norm_pre.resize(feats_down_size);
        {
            // 这个循环主要是把点转到imu坐标系并存入，crossmat_list，耗时很短
            for (size_t i = 0; i < feats_down_body->size(); i++)
            {
                V3D point_this(feats_down_body->points[i].x,
                            feats_down_body->points[i].y,
                            feats_down_body->points[i].z);
                pbody_list[i]=point_this;
                
                // 将点云转换到imu位姿下 Pi = Til * Pl
                point_this = Lidar_R_wrt_IMU * point_this + Lidar_T_wrt_IMU;
                
                M3D point_crossmat;
                point_crossmat << SKEW_SYM_MATRX(point_this);// 取了个反对称矩阵
                crossmat_list[i]=point_crossmat;
            }
        }

        double clc_timestamp_update;{
            struct timespec time1 = {0};
            clock_gettime(CLOCK_REALTIME, &time1);
            clc_timestamp_update = time1.tv_sec + time1.tv_nsec*1e-9;
        }

        // 获取 ivox
        if(asyn_locmap){
            localmap_process_handle->get_ivox(ivox_);
        }

        double clc_timestamp_get_ivox;{
            struct timespec time1 = {0};
            clock_gettime(CLOCK_REALTIME, &time1);
            clc_timestamp_get_ivox = time1.tv_sec + time1.tv_nsec*1e-9;
        }


        /*** iterated state estimation ***/
        // output 模式
        double pcl_beg_time = Measures.lidar_beg_time;
        bool imu_upda_cov = false;
        effct_feat_num = 0;
        /**** point by point update ****/
        // 遍历点，进行状态更新
        idx = -1;
        for (k = 0; k < time_seq.size(); k++)
        {
            // point_body 为雷达坐标系下的点
            PointType &point_body  = feats_down_body->points[idx+time_seq[k]];

            // 当前点的时间戳
            time_current = point_body.curvature / 1000.0 + pcl_beg_time;

            // 对于第一个数据，主要用来更新初始数据
            // 把当前点的时间戳之前的 li_int->imu_deque 数据全部删除，并更新 angvel_avr、acc_avr 值为刚好小于当前点时间戳的imu数据
            // 这个完成后，下面的 while (imu_comes) 
            if (is_first_frame)
            {
                // 寻找到时间戳刚好大于雷达初始点的的imu数据
                // 此举可以保证第一帧imu数据与雷达数据对齐
                while ((!li_int->imu_deque.empty()))
                {
                    double imu_time = rclcpp::Time(li_int->imu_deque.front()->header.stamp).seconds(); 
                    if(imu_time > time_current) break;
                    li_int->imu_deque.pop_front();
                }
                is_first_frame = false;
                imu_upda_cov = true;
                time_update_last = time_current;
                time_predict_last_const = time_current;
            }
            
            // 雷达数据时间戳大于imu数据时间戳，则表示改 imu 数据没有处理过，进行状态更新和 imu 原始数据观测
            bool imu_comes = time_current > rclcpp::Time(li_int->imu_deque.front()->header.stamp).seconds();
            while (imu_comes) 
            {
                imu_upda_cov = true;
                angvel_avr<<li_int->imu_deque.front()->angular_velocity.x, li_int->imu_deque.front()->angular_velocity.y, li_int->imu_deque.front()->angular_velocity.z;
                acc_avr   <<li_int->imu_deque.front()->linear_acceleration.x, li_int->imu_deque.front()->linear_acceleration.y, li_int->imu_deque.front()->linear_acceleration.z;

                /*** covariance update ***/
                // 名义状态转移
                double dt = rclcpp::Time(li_int->imu_deque.front()->header.stamp).seconds() - time_predict_last_const;
                // std::cout << "li_int->imu_deque.front() dt: " << dt << std::endl;
                kf_output.predict(dt, Q_output, input_in, true, false);
                time_predict_last_const = rclcpp::Time(li_int->imu_deque.front()->header.stamp).seconds();
                
                double dt_cov = rclcpp::Time(li_int->imu_deque.front()->header.stamp).seconds() - time_update_last; 

                if (dt_cov > 0.0)
                {
                    time_update_last = rclcpp::Time(li_int->imu_deque.front()->header.stamp).seconds();
                    double propag_imu_start = omp_get_wtime();

                    // 误差状态转移，这一步如果以雷达的频率进行计算会相当耗时，这里以imu的频率更新以比较低的频率运行节省计算资源
                    // 把这一步放到雷达数据状态转移之后，实际效果会更好
                    kf_output.predict(dt_cov, Q_output, input_in, false, true);

                    double solve_imu_start = omp_get_wtime();

                    // imu原始数据观测
                    kf_output.update_iterated_dyn_share_IMU();

                    propag_time += omp_get_wtime() - propag_imu_start;
                    // solve_time += omp_get_wtime() - solve_imu_start;
                }
                    
                li_int->imu_deque.pop_front();
                if (li_int->imu_deque.empty()) break;
                imu_comes = time_current > rclcpp::Time(li_int->imu_deque.front()->header.stamp).seconds();
            }
            

            double dt = time_current - time_predict_last_const;
            // 若无新的需要更新状态的imu数据，则进行名义状态更新，这样子可以保证得到当前点的对应位姿（相当于可以去畸变用）
            // 剧烈运动状态下，及时跟新状态并用最新状态更新来查找最邻近点很重要，可以保证在剧烈运动下的鲁棒性
            kf_output.predict(dt, Q_output, input_in, true, false);
            time_predict_last_const = time_current;


            double t_update_start = omp_get_wtime();
            if (feats_down_size < 1)
            {
                printf("No point, skip this scan!\n");
                idx += time_seq[k];
                continue;
            }
            double solve_start = omp_get_wtime();

            // 点云观测，有可能出现点云的 effct_feat_num=0 的情况，这种情况直接退出此次优化循环就好
            if (!kf_output.update_iterated_dyn_share_modified()) 
            {
                idx = idx+time_seq[k];
                continue;
            }
            solve_time += omp_get_wtime() - solve_start;

            for (int j = 0; j < time_seq[k]; j++)
            {
                PointType &point_body_j  = feats_down_body->points[idx+j+1];
                PointType &point_world_j = feats_down_world->points[idx+j+1];
                pointBodyToWorld(&point_body_j, &point_world_j);
            }
        
            idx += time_seq[k];
            // cout << "pbp output effect feat num:" << effct_feat_num << endl;
        }
        cout << "pbp output effect feat num:" << effct_feat_num << endl;


        /******* Publish odometry downsample *******/
        publish_odometry();

        /*** add the feature points to map ***/
        double clc_timestamp_map;{
            struct timespec time1 = {0};
            clock_gettime(CLOCK_REALTIME, &time1);
            clc_timestamp_map = time1.tv_sec + time1.tv_nsec*1e-9;
        }
        if(feats_down_size > 4)
        {
            Eigen::Vector3d pose = kf_output.x_.pos;
            PointType pose_pt; pose_pt.x = pose[0]; pose_pt.y = pose[1]; pose_pt.z = pose[2]; 
            if(asyn_locmap){
                localmap_process_handle->set_currentpts(*feats_down_world, pose_pt, lidar_end_time);
            }else{
                ivox_->AddPoints(feats_down_world->points);
                // ivox_->CutKeys(pose_pt, 50.0);
            }
        }

        /******* Publish points *******/
        publish_frame_world();

        double clc_timestamp_endl;{
            struct timespec time1 = {0};
            clock_gettime(CLOCK_REALTIME, &time1);
            clc_timestamp_endl = time1.tv_sec + time1.tv_nsec*1e-9;
        }


        std::cout << "h2_time: " << h2_time << std::endl;
        std::cout << "serch_time: " << serch_time << std::endl;
        h2_time = 0.0;
        serch_time = 0.0;
        // std::cout << "preprocess_t: " << clc_timestamp_getclose-clc_timestamp_start << std::endl; // clc_timestamp_getclose
        // std::cout << "getclose_t: " << clc_timestamp_update-clc_timestamp_getclose << std::endl;
        std::cout << "propag_time: " << propag_time << std::endl;
        std::cout << "get_ivox_t: " << clc_timestamp_get_ivox-clc_timestamp_update << std::endl;
        std::cout << "update_t: " << clc_timestamp_map-clc_timestamp_update << std::endl;
        std::cout << "solve_time: " << solve_time << std::endl;
        std::cout << "map_incremental_t: " << clc_timestamp_endl-clc_timestamp_map << std::endl;
        std::cout << "compute_t: " << clc_timestamp_endl-clc_timestamp_start << std::endl;
        std::cout << "-----\n\n " << std::endl;

        usleep(2000);
        // sleep(1);
        std::cout << "test" << std::endl;
    }

}


// int main(int argc, char** argv)
// {
//     // 设置固定核心
//     {
//         cpu_set_t cpuset;
//         CPU_ZERO(&cpuset);
//         CPU_SET(1, &cpuset);
//         if(0 != pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset)){
//             std::cout<<"Error: Set affinity failed in main."<<std::endl;
//             exit(-2);
//         }
//     }

//     YAML::Node config = YAML::LoadFile("/home/pf/LJJ_SRC/lio/point_lio/ws_grid/Point-LIO-point-lio-with-grid-map/config/avia.yaml");
//     readParameters(config);

//     rclcpp::init(0, nullptr);
//     // 创建 ROS2 节点
//     rclcpp::Node::SharedPtr nh_ = std::make_shared<rclcpp::Node>("laser_mapping");

//     laserMapping *lio = new laserMapping(nh_);
//     std::thread laserMapping_process_thread = std::thread(&laserMapping::run, lio);
//     // 设置固定核心
//     {
//         cpu_set_t cpuset;
//         CPU_ZERO(&cpuset);
//         CPU_SET(2, &cpuset);
//         if(0 != pthread_setaffinity_np(laserMapping_process_thread.native_handle(), sizeof(cpu_set_t), &cpuset)){
//             std::cout<<"Error: Set affinity failed in laserMapping::run."<<std::endl;
//             exit(-2);
//         }
//     }

//     LocalMapProcess *localmap_process_handle_ = new LocalMapProcess();
//     lio->setLocalMapHandle(localmap_process_handle_);
//     std::thread *localmap_process_thread; 
//     if(asyn_locmap){
//         localmap_process_thread = new std::thread(&LocalMapProcess::run, localmap_process_handle_);
//         // 设置固定核心
//         {
//             cpu_set_t cpuset;
//             CPU_ZERO(&cpuset);
//             CPU_SET(2, &cpuset);
//             if(0 != pthread_setaffinity_np(localmap_process_thread->native_handle(), sizeof(cpu_set_t), &cpuset)){
//                 std::cout<<"Error: Set affinity failed in MapProcess::run."<<std::endl;
//                 exit(-2);
//             }
//         } 
//     }

//     rclcpp::spin(nh_);

//     rclcpp::shutdown();
//     return 0;

// //     ros::init(argc, argv, "laserMapping");
// //     ros::NodeHandle nh("~");
// //     ros::AsyncSpinner spinner(0);
// //     spinner.start();    

// //     /*** variables definition and load Parameters***/
// //     YAML::Node config = YAML::LoadFile("/home/pf/LJJ_SRC/lio/point_lio/ws_grid/Point-LIO-point-lio-with-grid-map/config/avia.yaml");
// //     readParameters(config);
// //     int frame_num = 0;
// //     memset(point_selected_surf, true, sizeof(point_selected_surf));
// //     Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
// //     Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
// //     cout<<"lidar_type: "<<p_pre->lidar_type<<endl;
// //     ivox_ = std::make_shared<IVoxType>(ivox_options_);
// //     G_m_s2 = std::sqrt(gravity[0] * gravity[0] + gravity[1] * gravity[1] + gravity[2] * gravity[2]);
// //     p_imu->lidar_type = p_pre->lidar_type = lidar_type;
// //     p_imu->imu_en = imu_en;
// //     p_imu->G_m_s2 = G_m_s2;
    
// //     bool init_map = false, flg_first_scan = true;
// //     // Time Log Variables
// //     double match_time = 0, solve_time = 0, propag_time = 0;
// //     //surf feature in map
// //     PointCloudXYZI::Ptr init_feats_world(new PointCloudXYZI());
// //     li_initialization li_int;
// //     MeasureGroup Measures;


// //     // 设置固定核心
// //     {
// //         cpu_set_t cpuset;
// //         CPU_ZERO(&cpuset);
// //         CPU_SET(1, &cpuset);
// //         if(0 != pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset)){
// //             std::cout<<"Error: Set affinity failed in main."<<std::endl;
// //             exit(-2);
// //         }
// //     }

// //     // MapProcess map_process;
// //     // Eigen::Vector3d pose_last = {0.0, 0.0, 0.0};
// //     // std::thread* map_process_thread = new std::thread(&MapProcess::run, &map_process); 


// //     LocalMapProcess *localmap_process_handle = new LocalMapProcess(nh);
// //     if(asyn_locmap){
// //         std::thread localmap_process_thread = std::thread(&LocalMapProcess::run, localmap_process_handle);

// //         // 设置固定核心
// //         {
// //             cpu_set_t cpuset;
// //             CPU_ZERO(&cpuset);
// //             CPU_SET(2, &cpuset);
// //             if(0 != pthread_setaffinity_np(localmap_process_thread.native_handle(), sizeof(cpu_set_t), &cpuset)){
// //                 std::cout<<"Error: Set affinity failed in MapProcess::run."<<std::endl;
// //                 exit(-2);
// //             }
// //         } 
// //     }


// //     /*** eskf initialization ***/
// // // 30维
// // // MTK_BUILD_MANIFOLD(state_output, 
// // // ((vect3, pos))
// // // ((SO3, rot))
// // // ((SO3, offset_R_L_I))
// // // ((vect3, offset_T_L_I))
// // // ((vect3, vel))
// // // ((vect3, omg))
// // // ((vect3, acc))
// // // ((vect3, gravity))
// // // ((vect3, bg))
// // // ((vect3, ba))
// // // );

// // // 这个在 kf_output 模式下没有用（直接从状态向量里取了），在kf_input模式下需要用到，在状态转移的时候提供imu的数据
// // // MTK_BUILD_MANIFOLD(input_ikfom,
// // // ((vect3, acc))
// // // ((vect3, gyro))
// // // );

// //     /* 
// //     初始化卡尔曼滤波器相关，此卡尔曼滤波为误差状态卡尔曼滤波 ESKF（ESEKF） 形式
// //     get_f_output: 名义状态递推得到预测状态 vel、omg、acc，对应论文的 f(x,w) = [v, w, R.a+g, 0(3*1), 0(3*1), 0(3*1), 0(3*1), 0(3*1)]，PVR之外的其它状态保持不变
// //                     kf_output.predict(dt, Q_output, input_in, true, false);时被调用 f(x_, i_in)
// //     df_dx_output: 误差状态转移，根据误差状态的雅可比矩阵，得到误差状态预测
// //                     kf_output.predict(dt, Q_output, input_in, false, true);时被调用 f_x(x_, i_in)
// //     h_model_output: 雷达观测，主要观测位姿，其残差是根据当前定位的3D点坐标和地图点坐标的差值，，H矩阵参照论文。satu_check保证在没有点面匹配时不做观测
// //                     kf_output.update_iterated_dyn_share_IMU();时被调用 h_dyn_share_modified_2(x_, dyn_share)
// //     h_model_IMU_output: imu原始数据观测，其残差比较直接（z = angvel_avr - s.omg）上一帧数据与当前帧数据的差值，H矩阵参照论文。satu_check保证在加速度计不正常时不做观测
// //                     kf_output.update_iterated_dyn_share_IMU();时被调用 h_dyn_share_modified_1(x_, dyn_share);
// //     */
// //     kf_output.init_dyn_share_modified_3h(get_f_output, df_dx_output, h_model_output, h_model_IMU_output);
// //     Eigen::Matrix<double, 30, 30> P_init_output; // = MD(24, 24)::Identity() * 0.01;
// //     reset_cov_output(P_init_output);
// //     kf_output.change_P(P_init_output);
// //     Eigen::Matrix<double, 30, 30> Q_output = process_noise_cov_output();
    

// //     /*** ROS subscribe initialization ***/
// //     ros::Subscriber sub_pcl = p_pre->lidar_type == AVIA ? \
// //         nh.subscribe(lid_topic, 200000, &li_initialization::livox_pcl_cbk, &li_int) : \
// //         nh.subscribe(lid_topic, 200000, &li_initialization::standard_pcl_cbk, &li_int);
// //     ros::Subscriber sub_imu = nh.subscribe(imu_topic, 200000, &li_initialization::imu_cbk, &li_int);
// //     ros::Subscriber sub_reset = nh.subscribe("/reset", 1, &li_initialization::reset_cbk, &li_int);
// //     ros::Subscriber sub_reloc = nh.subscribe("/initialpose", 1, &li_initialization::initialpose_callback, &li_int);

// //     ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>
// //             ("/cloud_registered", 1000); // 发布世界坐标系下的点云（降采样后）
// //     ros::Publisher pubLaserCloudFullRes_body = nh.advertise<sensor_msgs::PointCloud2>
// //             ("/cloud_registered_body", 1000, true); // 发布test
// //     ros::Publisher pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>
// //             ("/Laser_map", 1000, true); // 发布ivox地图
// //     ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry> 
// //             ("/aft_mapped_to_init", 1000);
// //     ros::Publisher pubPath          = nh.advertise<nav_msgs::Path> 
// //             ("/path", 1000);
// //     li_int.pubOdom_imufreq = nh.advertise<nav_msgs::Odometry> 
// //             ("/Odom_imufreq", 1000);


// //     PointCloudXYZI::Ptr Global_map_load(new PointCloudXYZI());
// //     std::vector<bool> Global_map_bool_vector;
// //     if(!asyn_locmap){
// //         if(pure_loc){
// //             // load map:
// //             pcl::io::loadPCDFile(map_path, *Global_map_load);
// //             Global_map_bool_vector.resize(Global_map_load->size(), false);

// //             std::cout << "Global_map_load->size(): " << Global_map_load->size() << std::endl;

// //             sensor_msgs::PointCloud2 laserCloudmsg;
// //             pcl::toROSMsg(*Global_map_load, laserCloudmsg);
// //             laserCloudmsg.header.stamp = ros::Time::now();
// //             laserCloudmsg.header.frame_id = "camera_init";
// //             pubLaserCloudMap.publish(laserCloudmsg);
// //         }
// //         std::cout << "Global_map load succeed" << std::endl;
// //     }


// // //------------------------------------------------------------------------------------------------------
// //     // signal(SIGINT, SigHandle);
// //     ros::Rate loop_rate(500);
// //     while (ros::ok())
// //     {
// //         ros::spinOnce();

// //         // 状态复位操作
// //         if (li_int.reset_flag)
// //         {
// //             // 获取局部地图
// //             if(!pure_loc && ivox_->NumValidGrids() > 1){
// //                 std::vector<PointType> points_all = ivox_->GetAllPoints();
// //                 std::cout << "points_all.size(): " << points_all.size() << std::endl;
// //                 pcl::PointCloud<pcl::PointXYZINormal> clouds_local;
// //                 // 批量转换 std::vector 到 pcl::PointCloud
// //                 clouds_local.points.insert(clouds_local.points.end(), points_all.begin(), points_all.end());

// //                 sensor_msgs::PointCloud2 laserCloudmsg;
// //                 pcl::toROSMsg(clouds_local, laserCloudmsg);
// //                 laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
// //                 laserCloudmsg.header.frame_id = "camera_init";
// //                 pubLaserCloudMap.publish(laserCloudmsg);

// //                 std::cout << "save map to root_dir: " << root_dir << std::endl;
// //                 pcl::io::savePCDFileBinary(root_dir + "map.pcd", clouds_local);
// //             }

// //             std::cout << "--------------- reset slam system ---------------\n\n\n\n\n" << std::endl;


// //             param_reset();
// //             YAML::Node config = YAML::LoadFile("/home/pf/LJJ_SRC/lio/point_lio/ws_grid/Point-LIO-point-lio-with-grid-map/config/avia.yaml");
// //             readParameters(config);
            
// //             p_imu->Reset();
// //             kf_output.change_P(P_init_output);
                
// //             flg_first_scan = true;
// //             is_first_frame = true;
// //             init_map = false;

            
// //             init_feats_world = PointCloudXYZI::Ptr (new PointCloudXYZI());

// //             Measures = MeasureGroup();


// //             est_reset();

// //             li_int.reset();
// //             li_int.reset_flag = false;

// //             path.poses.clear();

// //             if(asyn_locmap){
// //                 localmap_process_handle->Reset();
// //             }
// //         }


// //         // Measures为输出，输入为lid_topic和imu_topic消息回调得到的数据队列
// //         // 而其实后面处理，除了初始化的时候用Measures的imu数据，其它情况下都是直接操作imu数据的队列......
// //         // 且，Measures的imu数据也没有更新过......
// //         // Measures为输出，输入为lid_topic和imu_topic消息回调得到的数据队列
// //         // 而其实后面处理，除了初始化的时候用Measures的imu数据，其它情况下都是直接操作imu数据的队列......
// //         // 且，Measures的imu数据也没有更新过......
// //         if(!li_int.sync_packages(Measures)) {
// //             usleep(2000);
// //             continue;
// //         }

// //         double clc_timestamp_start;{
// //             struct timespec time1 = {0};
// //             clock_gettime(CLOCK_REALTIME, &time1);
// //             clc_timestamp_start = time1.tv_sec + time1.tv_nsec*1e-9;
// //         }


// //         match_time = 0;
// //         solve_time = 0;
// //         propag_time = 0;
        
// //         /*** downsample the feature points in a scan ***/
// //         // 主要是针对初 imu 始化，得到 imu 初始均值，这个会运行几次，MAX_INI_COUNT 参数代表需要的 imu 数据量
// //         // 完成后才会使 p_imu->imu_need_init_ = false;
// //         // 在 imu_need_init_ 的时候，会让此次循环退出，以收集更多的imu数据
// //         // 确保初始化的时候静止1-2s
// //         if (p_imu->imu_need_init_){
// //             p_imu->Process(Measures);
// //             continue;
// //         }


// //         // 确定重力大小和初始方向
// //         // 并设置卡尔曼滤波状态变量的方向相关状态
// //         // p_imu->gravity_align_ 在初始化时为 false ，代表初始化状态、且 imu 数据充足的情况
// //         // 注意 state_out 只在这里起作用，目的是总结数据给 kf_output.change_x(state_out);
// //         if (!p_imu->gravity_align_)
// //         {
// //             state_output state_out;
// //             // 注意单位是g还是m/s^2，这里是把p_imu->mean_acc 单位是 g、state_out.gravity单位是m/s^2
// //             state_out.gravity = -1 * p_imu->mean_acc * G_m_s2 / acc_norm; 
// //             state_out.acc = p_imu->mean_acc * G_m_s2 / acc_norm;

// //             // 确保初始化的时候静止1-2s
// //             {
// //                 // 根据当前加速度计数据 state_out.acc 和重力向量 p_imu->gravity_ ，计算 Rag = rot_init
// //                 Eigen::Matrix3d rot_init;
// //                 // p_imu->gravity_ 只在这个时候有用
// //                 p_imu->gravity_ << VEC_FROM_ARRAY(gravity);
// //                 p_imu->Set_init(state_out.acc, rot_init); // 得到 rot_init 为 Rag
// //                 std::cout << "rot_init(Rga): \n " << rot_init.transpose() << std::endl;
// //                 Eigen::Vector3d euler_xyz = rot_init.transpose().eulerAngles(0, 1, 2);
// //                 std::cout << "INIT Euler angles (XYZ order)(Rag):\n" 
// //                         << "Roll (X): " << euler_xyz[0]*180/3.14 << " deg\n" 
// //                         << "Pitch (Y): " << euler_xyz[1]*180/3.14 << " deg\n" 
// //                         << "Yaw (Z): " << euler_xyz[2]*180/3.14 << " deg\n";

// //                 // 更新初始状态量
// //                 state_out.gravity = p_imu->gravity_;
// //                 state_out.rot = rot_init.transpose();
                
// //                 // 根据旋转矩阵得到当前加速度数据（而且应该不用transpose吧），这里因为重力是向下的与加速度计测量相反，所以前面加负号
// //                 // 多此一举？
// //                 state_out.acc = rot_init * (-state_out.gravity);
// //             }
// //             kf_output.change_x(state_out);
// //             p_imu->gravity_align_ = true;

// //             // std::cout << "state_out.rot: \n" << state_out.rot << std::endl;
// //             // std::cout << "state_out.gravity: " << state_out.gravity << std::endl;
// //             // std::cout << "state_out.acc: " << state_out.acc << std::endl;

// //             // std::cout << "kf_output.x_.rot: \n" << kf_output.x_.rot << std::endl;
// //             // std::cout << "kf_output.x_.acc: " << kf_output.x_.acc << std::endl;
// //             // std::cout << "kf_output.x_.gravity: " << kf_output.x_.gravity << std::endl;
// //         }


// //         /*** initialize the map ***/
// //         // 确保初始化的时候静止1-2s
// //         if(!init_map)
// //         {
// //             feats_down_world->resize(Measures.lidar->size());            
// //             // 把点转到世界坐标系（重力对齐）
// //             for(int i = 0; i < Measures.lidar->size(); i++)
// //             {
// //                 pointBodyToWorld(&(Measures.lidar->points[i]), &(feats_down_world->points[i]));
// //             }
// //             // 存入点到init_feats_world
// //             for (size_t i = 0; i < feats_down_world->size(); i++) 
// //             {
// //                 init_feats_world->points.emplace_back(feats_down_world->points[i]);
// //             }
// //             // 如果点数不够则继续往地图里加点
// //             if(init_feats_world->size() < init_map_size){
// //                 init_map = false;
// //             }else{   
// //                 // 手动重定位
// //                 if(pure_loc){
// //                     Eigen::Matrix4d T_ = Eigen::Matrix4d::Identity();

// //                     {
// //                     sensor_msgs::PointCloud2 laserCloudmsg;
// //                     pcl::toROSMsg(*init_feats_world, laserCloudmsg);
// //                     laserCloudmsg.header.stamp = ros::Time::now();
// //                     laserCloudmsg.header.frame_id = "camera_init";
// //                     pubLaserCloudFullRes_body.publish(laserCloudmsg);
// //                     }

// //                     if(li_int.manual_reloc_flag){
// //                         li_int.manual_reloc_flag = false;

// //                         std::cout << " use maul reloc relocation: \n "<< li_int.manual_reloc_pose << std::endl;
// //                         std::cout << " init_feats_world->size(): "<< init_feats_world->size() << std::endl;

// //                         // ResetLocalMap
// //                         PointCloudXYZI::Ptr Local_map_(new PointCloudXYZI());
// //                         Local_map_->points.reserve(50000);
// //                         if(asyn_locmap){
// //                             std::fill(localmap_process_handle->Global_map_bool_vector.begin(), localmap_process_handle->Global_map_bool_vector.end(), false);
// //                             for (int i = 0; i < localmap_process_handle->Global_map_load->points.size(); i++){
// //                                 if (abs(localmap_process_handle->Global_map_load->points[i].x - li_int.manual_reloc_pose(0, 3)) > 100 ||
// //                                     abs(localmap_process_handle->Global_map_load->points[i].y - li_int.manual_reloc_pose(1, 3)) > 100 ||
// //                                     abs(localmap_process_handle->Global_map_load->points[i].z - li_int.manual_reloc_pose(2, 3)) > 100){
// //                                 continue;
// //                                 }
                                
// //                                 PointType p;
// //                                 p.x = localmap_process_handle->Global_map_load->points[i].x;
// //                                 p.y = localmap_process_handle->Global_map_load->points[i].y;
// //                                 p.z = localmap_process_handle->Global_map_load->points[i].z;
// //                                 p.intensity = localmap_process_handle->Global_map_load->points[i].intensity;
// //                                 // localmap_process_handle->Global_map_bool_vector[i] = true;
// //                                 Local_map_->points.emplace_back(p);
// //                             }
// //                         }else{
// //                             std::fill(Global_map_bool_vector.begin(), Global_map_bool_vector.end(), false);
// //                             for (int i = 0; i < Global_map_load->points.size(); i++){
// //                                 if (abs(Global_map_load->points[i].x - li_int.manual_reloc_pose(0, 3)) > 100 ||
// //                                     abs(Global_map_load->points[i].y - li_int.manual_reloc_pose(1, 3)) > 100 ||
// //                                     abs(Global_map_load->points[i].z - li_int.manual_reloc_pose(2, 3)) > 100){
// //                                 continue;
// //                                 }
                                
// //                                 PointType p;
// //                                 p.x = Global_map_load->points[i].x;
// //                                 p.y = Global_map_load->points[i].y;
// //                                 p.z = Global_map_load->points[i].z;
// //                                 p.intensity = Global_map_load->points[i].intensity;
// //                                 Global_map_bool_vector[i] = true;
// //                                 Local_map_->points.emplace_back(p);
// //                             }
// //                         }
// //                         std::cout << " Local_map_->size(): "<< Local_map_->size() << std::endl;
                        
// //                         {
// //                             sensor_msgs::PointCloud2 Local_mapmsg;
// //                             pcl::toROSMsg(*Local_map_, Local_mapmsg);
// //                             Local_mapmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
// //                             Local_mapmsg.header.frame_id = "camera_init";
// //                             pubLaserCloudMap.publish(Local_mapmsg);
// //                         }




// //                         PointCloudXYZI::Ptr output(new PointCloudXYZI());
// //                         pcl::NormalDistributionsTransform<PointType, PointType> ndt;
// //                         ndt.setTransformationEpsilon(1e-8);
// //                         ndt.setTransformationRotationEpsilon(1e-8);
// //                         ndt.setStepSize(0.7); // maximum step length
// //                         ndt.setMaximumIterations(40);
// //                         ndt.setResolution(1.0);
// //                         ndt.setInputTarget(Local_map_);
// //                         ndt.setInputSource(init_feats_world);
// //                         ndt.align(*output, li_int.manual_reloc_pose.cast<float>());
// //                         double ndt_score_ = ndt.getTransformationProbability();

// //                         std::cout << "ndt_Itera_: " << ndt.getFinalNumIteration() << std::endl;
// //                         std::cout << "ndt_score_: " << ndt_score_ << std::endl; 
// //                         std::cout << "ndt.getFinalTransformation(): \n" << ndt.getFinalTransformation() << std::endl;

// //                         // ndt 得分跟它设置的分辨率有关，1.0分辨率下 score大于4.0合适
// //                         if(ndt_score_ > 3.0){
// //                             // 再来一遍gicp提高初始化精度
// //                             pcl::GeneralizedIterativeClosestPoint<PointType, PointType> gicp;
// //                             gicp.setInputSource(init_feats_world);
// //                             gicp.setInputTarget(Local_map_);
// //                             gicp.setCorrespondenceRandomness(10); // 设置选择点邻域时使用的邻居数量
// //                             gicp.setMaximumIterations(50);       // 设置最大迭代次数
// //                             gicp.setTransformationEpsilon(1e-8); // 设置转换精度
// //                             gicp.setTransformationRotationEpsilon(1e-8);
// //                             gicp.setMaxCorrespondenceDistance(5.0); // 设置对应点对的最大距离
// //                             // 存储配准结果
// //                             PointCloudXYZI::Ptr cloud_aligned(new PointCloudXYZI());
// //                             gicp.align(*cloud_aligned , ndt.getFinalTransformation());

// //                             // 输出配准信息
// //                             if (gicp.hasConverged()) {
// //                                 T_ = gicp.getFinalTransformation().cast<double>();
// //                                 std::cout << "GICP配准成功!" << std::endl;
// //                                 std::cout << "配准分数: " << gicp.getFitnessScore() << std::endl; // 配准后每对对应点之间的欧氏距离。分数参考意义不大，很难判断匹配结果可靠性
// //                                 std::cout << "T_: \n" << T_ << std::endl;

// //                                 // 把点转到世界坐标系，并显示
// //                                 {                        
// //                                     for(int i = 0; i < init_feats_world->size(); i++)
// //                                     {
// //                                         Eigen::Vector3d p_pre(init_feats_world->points[i].x, init_feats_world->points[i].y, init_feats_world->points[i].z);
// //                                         Eigen::Vector3d p_aft;
// //                                         p_aft = T_.block<3, 3>(0, 0) * p_pre + T_.block<3, 1>(0, 3);

// //                                         init_feats_world->points[i].x = p_aft(0);
// //                                         init_feats_world->points[i].y = p_aft(1);
// //                                         init_feats_world->points[i].z = p_aft(2);
// //                                     }

// //                                     sensor_msgs::PointCloud2 laserCloudmsg;
// //                                     pcl::toROSMsg(*init_feats_world, laserCloudmsg);
// //                                     laserCloudmsg.header.stamp = ros::Time::now();
// //                                     laserCloudmsg.header.frame_id = "camera_init";
// //                                     pubLaserCloudFullRes_body.publish(laserCloudmsg);
// //                                 }
// //                             } else {
// //                                 std::cout << "GICP配准失败!" << std::endl;
// //                                 continue;
// //                             }
// //                         }else{
// //                             std::cout << "NDT配准失败!" << std::endl;
// //                             continue;
// //                         }


// //                         /*
// //                         // 设置GICP配准对象，gicp精度更高，但问题在于不好评估匹配结果是否可信
// //                         pcl::GeneralizedIterativeClosestPoint<PointType, PointType> gicp;
// //                         gicp.setInputSource(init_feats_world);
// //                         gicp.setInputTarget(Local_map_);
// //                         gicp.setCorrespondenceRandomness(10); // 设置选择点邻域时使用的邻居数量
// //                         gicp.setMaximumIterations(50);       // 设置最大迭代次数
// //                         gicp.setTransformationEpsilon(1e-8); // 设置转换精度
// //                         gicp.setTransformationRotationEpsilon(1e-8);
// //                         gicp.setMaxCorrespondenceDistance(5.0); // 设置对应点对的最大距离
// //                         // 存储配准结果
// //                         PointCloudXYZI::Ptr cloud_aligned(new PointCloudXYZI());
// //                         gicp.align(*cloud_aligned , li_int.manual_reloc_pose.cast<float>());//
// //                         // 输出配准信息
// //                         if (gicp.hasConverged() && gicp.getFitnessScore() < 0.1) {
// //                             std::cout << "GICP配准成功!" << std::endl;
// //                             std::cout << "配准分数: " << gicp.getFitnessScore() << std::endl; // 配准后每对对应点之间的欧氏距离。分数参考意义不大，很难判断匹配结果可靠性
// //                             std::cout << "变换矩阵: \n" << gicp.getFinalTransformation() << std::endl;
// //                             Eigen::Matrix4d T_ = gicp.getFinalTransformation().cast<double>();
// //                             std::cout << "T_: \n" << T_ << std::endl;

// //                             // 把点转到世界坐标系，并显示
// //                             {                        
// //                                 for(int i = 0; i < init_feats_world->size(); i++)
// //                                 {
// //                                     Eigen::Vector3d p_pre(init_feats_world->points[i].x, init_feats_world->points[i].y, init_feats_world->points[i].z);
// //                                     Eigen::Vector3d p_aft;
// //                                     p_aft = T_.block<3, 3>(0, 0) * p_pre + T_.block<3, 1>(0, 3);

// //                                     init_feats_world->points[i].x = p_aft(0);
// //                                     init_feats_world->points[i].y = p_aft(1);
// //                                     init_feats_world->points[i].z = p_aft(2);
// //                                 }

// //                                 sensor_msgs::PointCloud2 laserCloudmsg;
// //                                 pcl::toROSMsg(*init_feats_world, laserCloudmsg);
// //                                 laserCloudmsg.header.stamp = ros::Time::now();
// //                                 laserCloudmsg.header.frame_id = "camera_init";
// //                                 pubLaserCloudFullRes_body.publish(laserCloudmsg);
// //                             }

// //                             li_int.manual_reloc_flag = false;
// //                             break;
// //                         } else {
// //                             std::cout << "GICP配准失败!" << std::endl;
// //                         }
// //                         */
                    
                        
// //                         // 手动初始化完成后，更新eskf的状态、根据初始位置构建ivox
// //                         state_output state_out;
// //                         state_out.gravity = p_imu->gravity_;
// //                         state_out.rot = T_.block<3, 3>(0, 0)*kf_output.x_.rot;
// //                         state_out.pos = T_.block<3, 1>(0, 3);
// //                         kf_output.change_x(state_out);
// //                         std::cout << "kf_output changed in reloc" << std::endl;

// //                         ivox_->AddPoints(Local_map_->points);
// //                     }else{
// //                         if(init_feats_world->size() > init_map_size){
// //                             init_feats_world.reset(new PointCloudXYZI());
// //                         }
// //                         continue;
// //                     }
// //                 }


// //                 init_map = true;
// //             }
// //             // init_map流程可能会存有部分lidar数据缓存，把历史数据删除，重新对齐到最新帧处理
// //             // imu数据不做删除，会在is_first_frame时自己对齐
// //             if(init_map){
// //                 if(asyn_locmap){
// //                     localmap_process_handle->set_ivox(*ivox_);
// //                 }else{
// //                     ivox_->AddPoints(init_feats_world->points);
// //                 }

// //                 init_feats_world.reset(new PointCloudXYZI());
// //                 // // delay test
// //                 // sleep(5);
// //                 // std::cout << "lidar_buffer.size(): " << li_int.lidar_buffer.size() << std::endl;
// //                 // std::cout << "imu_deque.size(): " << li_int.imu_deque.size() << std::endl;
// //                 // 清空 deque
// //                 if (!li_int.lidar_buffer.empty()) {
// //                     li_int.lidar_buffer.clear();
// //                     li_int.imu_deque.clear();
// //                     li_int.time_buffer.clear();
// //                 }

// //                 std::cout << "-------- ivox_ map inited --------\n\n\n" << std::endl;
// //             }

// //             continue;
// //         }

// //         double clc_timestamp_getclose;{
// //             struct timespec time1 = {0};
// //             clock_gettime(CLOCK_REALTIME, &time1);
// //             clc_timestamp_getclose = time1.tv_sec + time1.tv_nsec*1e-9;
// //         }



// // ///////////////////////////////////////////////////////////////////////////////
// // ///// 初始化完成
// // ///////////////////////////////////////////////////////////////////////////////
// //         // 对当前帧雷达数据进行降采样，体素滤波，得到 feats_down_body ，后续处理都针对 feats_down_body
// //         // 默认分辨率5cm
// //         // 并按时间戳排序
// //         *feats_down_body = p_pre->down_sample(true, vf_scan_res_min, vf_max_num, vf_scan_res_max,
// //                                     vf_scan_res_step, Measures.lidar); 

// //         // 这一步很重要，对输入的点云数据进行时间压缩处理，基于点的 curvature 属性，将点云序列压缩成一系列的时间段。
// //         // time_seq 的维度代表若干个时间段内的点云数量（一般为1）
// //         // time_seq 内所存的元素为点云的索引
// //         time_seq = time_compressing<int>(feats_down_body); 
// //         int feats_down_size = feats_down_body->points.size();
// //         std::cout << "Measures.lidar->size(): " << Measures.lidar->size() << std::endl;
// //         std::cout << "feats_down_size: " << feats_down_size << std::endl;


// //         /*** ICP and Kalman filter update ***/
// //         // 这里就是初始化了三维点云的大小
// //         // 调用 resize() 后不会清除已有的数据，但会根据新大小对容器进行扩展或缩减
// //         // 但是在点云观测步骤 update_iterated_dyn_share_modified -> h_model_output 里，每个相关值都会被更新，与feats_down_body一一对应
// //         feats_down_world->clear();
// //         feats_down_world->resize(feats_down_size); 
// //         Nearest_Points.clear();
// //         Nearest_Points.resize(feats_down_size);
// //         crossmat_list.clear();
// //         crossmat_list.resize(feats_down_size);
// //         pbody_list.clear();
// //         pbody_list.resize(feats_down_size);
// //         pabcd_pre.clear();
// //         pabcd_pre.resize(feats_down_size);
// //         p_norm_pre.clear();
// //         p_norm_pre.resize(feats_down_size);

// //         double pcl_beg_time = Measures.lidar_beg_time;

        
// //         /*** iterated state estimation ***/

// //         {
// //             // 这个循环主要是把点转到imu坐标系并存入，crossmat_list，耗时很短
// //             for (size_t i = 0; i < feats_down_body->size(); i++)
// //             {
// //                 V3D point_this(feats_down_body->points[i].x,
// //                             feats_down_body->points[i].y,
// //                             feats_down_body->points[i].z);
// //                 pbody_list[i]=point_this;
                
// //                 // 将点云转换到imu位姿下 Pi = Til * Pl
// //                 point_this = Lidar_R_wrt_IMU * point_this + Lidar_T_wrt_IMU;
                
// //                 M3D point_crossmat;
// //                 point_crossmat << SKEW_SYM_MATRX(point_this);// 取了个反对称矩阵
// //                 crossmat_list[i]=point_crossmat;
// //             }
// //         }

// //         double clc_timestamp_update;{
// //             struct timespec time1 = {0};
// //             clock_gettime(CLOCK_REALTIME, &time1);
// //             clc_timestamp_update = time1.tv_sec + time1.tv_nsec*1e-9;
// //         }


// //         // 获取 ivox
// //         if(asyn_locmap){
// //             localmap_process_handle->get_ivox(ivox_);
// //         }

// //         double clc_timestamp_get_ivox;{
// //             struct timespec time1 = {0};
// //             clock_gettime(CLOCK_REALTIME, &time1);
// //             clc_timestamp_get_ivox = time1.tv_sec + time1.tv_nsec*1e-9;
// //         }

// //         // output 模式
// //         bool imu_upda_cov = false;
// //         effct_feat_num = 0;
// //         /**** point by point update ****/
// //         // 遍历点，进行状态更新
// //         idx = -1;
// //         for (k = 0; k < time_seq.size(); k++)
// //         {
// //             // point_body 为雷达坐标系下的点
// //             PointType &point_body  = feats_down_body->points[idx+time_seq[k]];

// //             // 当前点的时间戳
// //             time_current = point_body.curvature / 1000.0 + pcl_beg_time;

// //             // 对于第一个数据，主要用来更新初始数据
// //             // 把当前点的时间戳之前的 li_int.imu_deque 数据全部删除，并更新 angvel_avr、acc_avr 值为刚好小于当前点时间戳的imu数据
// //             // 这个完成后，下面的 while (imu_comes) 
// //             if (is_first_frame)
// //             {
// //                 // 寻找到时间戳刚好大于雷达初始点的的imu数据
// //                 // 此举可以保证第一帧imu数据与雷达数据对齐
// //                 while ((!li_int.imu_deque.empty()))
// //                 {
// //                     double imu_time = li_int.imu_deque.front()->header.stamp.toSec(); 
// //                     if(imu_time > time_current) break;
// //                     li_int.imu_deque.pop_front();
// //                 }
// //                 is_first_frame = false;
// //                 imu_upda_cov = true;
// //                 time_update_last = time_current;
// //                 time_predict_last_const = time_current;
// //             }
            
// //             // 雷达数据时间戳大于imu数据时间戳，则表示改 imu 数据没有处理过，进行状态更新和 imu 原始数据观测
// //             bool imu_comes = time_current > li_int.imu_deque.front()->header.stamp.toSec();
// //             while (imu_comes) 
// //             {
// //                 imu_upda_cov = true;
// //                 angvel_avr<<li_int.imu_deque.front()->angular_velocity.x, li_int.imu_deque.front()->angular_velocity.y, li_int.imu_deque.front()->angular_velocity.z;
// //                 acc_avr   <<li_int.imu_deque.front()->linear_acceleration.x, li_int.imu_deque.front()->linear_acceleration.y, li_int.imu_deque.front()->linear_acceleration.z;

// //                 /*** covariance update ***/
// //                 // 名义状态转移
// //                 double dt = li_int.imu_deque.front()->header.stamp.toSec() - time_predict_last_const;
// //                 // std::cout << "li_int.imu_deque.front() dt: " << dt << std::endl;
// //                 kf_output.predict(dt, Q_output, input_in, true, false);
// //                 time_predict_last_const = li_int.imu_deque.front()->header.stamp.toSec(); // big problem
                
// //                 double dt_cov = li_int.imu_deque.front()->header.stamp.toSec() - time_update_last; 

// //                 if (dt_cov > 0.0)
// //                 {
// //                     time_update_last = li_int.imu_deque.front()->header.stamp.toSec();
// //                     double propag_imu_start = omp_get_wtime();

// //                     // 误差状态转移，这一步如果以雷达的频率进行计算会相当耗时，这里以imu的频率更新以比较低的频率运行节省计算资源
// //                     // 把这一步放到雷达数据状态转移之后，实际效果会更好
// //                     kf_output.predict(dt_cov, Q_output, input_in, false, true);

// //                     double solve_imu_start = omp_get_wtime();

// //                     // imu原始数据观测
// //                     kf_output.update_iterated_dyn_share_IMU();

// //                     propag_time += omp_get_wtime() - propag_imu_start;
// //                     // solve_time += omp_get_wtime() - solve_imu_start;
// //                 }
                    
// //                 li_int.imu_deque.pop_front();
// //                 if (li_int.imu_deque.empty()) break;
// //                 imu_comes = time_current > li_int.imu_deque.front()->header.stamp.toSec();
// //             }
            

// //             double dt = time_current - time_predict_last_const;
// //             // 若无新的需要更新状态的imu数据，则进行名义状态更新，这样子可以保证得到当前点的对应位姿（相当于可以去畸变用）
// //             // 剧烈运动状态下，及时跟新状态并用最新状态更新来查找最邻近点很重要，可以保证在剧烈运动下的鲁棒性
// //             kf_output.predict(dt, Q_output, input_in, true, false);
// //             time_predict_last_const = time_current;


// //             double t_update_start = omp_get_wtime();
// //             if (feats_down_size < 1)
// //             {
// //                 ROS_WARN("No point, skip this scan!\n");
// //                 idx += time_seq[k];
// //                 continue;
// //             }
// //             double solve_start = omp_get_wtime();

// //             // 点云观测，有可能出现点云的 effct_feat_num=0 的情况，这种情况直接退出此次优化循环就好
// //             if (!kf_output.update_iterated_dyn_share_modified()) 
// //             {
// //                 idx = idx+time_seq[k];
// //                 continue;
// //             }
// //             solve_time += omp_get_wtime() - solve_start;

// //             for (int j = 0; j < time_seq[k]; j++)
// //             {
// //                 PointType &point_body_j  = feats_down_body->points[idx+j+1];
// //                 PointType &point_world_j = feats_down_world->points[idx+j+1];
// //                 pointBodyToWorld(&point_body_j, &point_world_j);
// //             }
        
// //             idx += time_seq[k];
// //             // cout << "pbp output effect feat num:" << effct_feat_num << endl;
// //         }
// //         cout << "pbp output effect feat num:" << effct_feat_num << endl;

// //         /* 定位状态评估
// //         1. 根据effct_feat_num，在100ms内的累计有效点小于2000点认为无效
// //         2. 对比imu解算的重力分量和定位结果的重力分量，差别太大认为无效
// //         3. 在加速度有效的情况下，使用加速度求解速度、定位速度对比，若差距过大认为无效
// //         */

// //        /* 退化判断
// //        异步处理
// //        收集1s内参与定位的点云，发送给异步线程
// //        异步线程里构建ivox并搜索所有点的最邻近，根据平面方程计算平面对XYZ轴的夹角，通过分析夹角，可以推测平面的方向性和对位姿约束的潜在贡献。
// //        平面与某坐标轴的夹角越接近 90°，说明平面与该坐标轴几乎正交，对该轴方向的几何贡献较少。
// //        并反馈到当前线程
// //        直接以世界坐标系为xy轴，与重力对齐后z方向固定、所以z方向确定可以最先判断并剔除，在剔除z轴后，以x分量明显大于y分量的和差距不明显的（可以根据比值确定角度）全划分到x，剩余的就是y轴主分量不用再计算
// //        若某一分量过少，则表示存在退化

// //        不异步方法，在正常场景里，根据每次定位的点的平面方程判断，并计数，1s内某一分量过少则表示存在退化
// //        */


// //         /******* Publish odometry downsample *******/
// //         if (!publish_odometry_without_downsample)
// //         {
// //             publish_odometry(pubOdomAftMapped);
// //         }

// //         /*** add the feature points to map ***/
// //         double clc_timestamp_map;{
// //             struct timespec time1 = {0};
// //             clock_gettime(CLOCK_REALTIME, &time1);
// //             clc_timestamp_map = time1.tv_sec + time1.tv_nsec*1e-9;
// //         }
// //         if(feats_down_size > 4)
// //         {
// //             Eigen::Vector3d pose = kf_output.x_.pos;
// //             PointType pose_pt; pose_pt.x = pose[0]; pose_pt.y = pose[1]; pose_pt.z = pose[2]; 
// //             if(asyn_locmap){
// //                 localmap_process_handle->set_currentpts(*feats_down_world, pose_pt, lidar_end_time);
// //             }else{
// //                 if(pure_loc && !Global_map_load->empty()){
// //                     pcl::PointCloud<PointType> increase_map_;
// //                     increase_map_.points.clear();
// //                     increase_map_.points.reserve(50000);
// //                     // ResetLocalMap
// //                     for (int i = 0; i < Global_map_load->size(); i++){
// //                         if (abs(Global_map_load->points[i].x - pose(0)) > 100 ||
// //                             abs(Global_map_load->points[i].y - pose(1)) > 100 ||
// //                             abs(Global_map_load->points[i].z - pose(2)) > 100){
// //                             Global_map_bool_vector[i] = false;
// //                             continue;
// //                         }


// //                         if(!Global_map_bool_vector[i]){
// //                             PointType p;
// //                             p.x = Global_map_load->points[i].x;
// //                             p.y = Global_map_load->points[i].y;
// //                             p.z = Global_map_load->points[i].z;
// //                             p.intensity = Global_map_load->points[i].intensity;
// //                             Global_map_bool_vector[i] = true;
// //                             increase_map_.points.emplace_back(p);
// //                         }
// //                     }
// //                     std::cout << " increase_map_.size(): "<< increase_map_.size() << std::endl;
// //                     ivox_->AddPoints(increase_map_.points);
// //                 }

// //                 ivox_->AddPoints(feats_down_world->points);
// //                 // ivox_->CutKeys(pose_pt, 50.0);
// //             }
// //         }

// //         /******* Publish points *******/
// //         publish_path(pubPath);
// //         publish_frame_world(pubLaserCloudFullRes);

// //         double clc_timestamp_endl;{
// //             struct timespec time1 = {0};
// //             clock_gettime(CLOCK_REALTIME, &time1);
// //             clc_timestamp_endl = time1.tv_sec + time1.tv_nsec*1e-9;
// //         }


// //         std::cout << "h2_time: " << h2_time << std::endl;
// //         std::cout << "serch_time: " << serch_time << std::endl;
// //         h2_time = 0.0;
// //         serch_time = 0.0;
// //         std::cout << "preprocess_t: " << clc_timestamp_getclose-clc_timestamp_start << std::endl; // clc_timestamp_getclose
// //         std::cout << "getclose_t: " << clc_timestamp_update-clc_timestamp_getclose << std::endl;
// //         std::cout << "propag_time: " << propag_time << std::endl;
// //         std::cout << "get_ivox_t: " << clc_timestamp_get_ivox-clc_timestamp_update << std::endl;
// //         std::cout << "update_t: " << clc_timestamp_map-clc_timestamp_update << std::endl;
// //         std::cout << "solve_time: " << solve_time << std::endl;
// //         std::cout << "map_incremental_t: " << clc_timestamp_endl-clc_timestamp_map << std::endl;
// //         std::cout << "compute_t: " << clc_timestamp_endl-clc_timestamp_start << std::endl;
// //         std::cout << "-----\n\n " << std::endl;


// //         int sunm_max = 10;
// //         static int count = 0;
// //         static int count_begin = 0;
// //         static double  compute_t_sum = 0.0;
// //         // 抛开前10帧的判断
// //         if(count_begin < 10){
// //             count_begin++;
// //         }else{
// //             // 若连续 sunm_max 帧的平均处理时间大于 process_t_max 则退出，耗时不正常
// //             if(count < sunm_max){
// //                 count++;
// //                 compute_t_sum += (clc_timestamp_endl-clc_timestamp_start);
// //             }else{
// //                 std::cout << "compute_t_sum: " << compute_t_sum << std::endl;
// //                 std::cout << "count: " << count << std::endl;
// //                 std::cout << "compute_t_sum/sunm_max: " << compute_t_sum/sunm_max << std::endl;
// //                 if(compute_t_sum/sunm_max > process_t_max){
// //                     exit(0);
// //                 }
// //                 compute_t_sum = 0.0;
// //                 count = 0;
// //             }
// //         }



// //         usleep(2000);
// //     }


//     return 0;
// }
