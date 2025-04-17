#include "li_initialization.h"


void li_initialization::standard_pcl_cbk(const std::shared_ptr<const sensor_msgs::msg::PointCloud2> msg) 
{
    double time_sec = rclcpp::Time(msg->header.stamp).seconds();

    if (time_sec < last_timestamp_lidar)
    {
        std::cout << "lidar loop back, clear buffer" << std::endl;
        lidar_buffer.shrink_to_fit();
        return;
    }
    // printf("lidar time: %.4f\n", time_sec);

    last_timestamp_lidar = time_sec;

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI(20000,1));
    p_pre->process(msg, ptr);
    // std::cout << "ptr->points.size(): " << ptr->points.size() << std::endl;
    if (ptr->points.size() > 0)
    {
        lidar_buffer.emplace_back(ptr);
        time_buffer.emplace_back(time_sec);
    }
}

// void li_initialization::livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg) 
// {
//     if (msg->header.stamp.toSec() < last_timestamp_lidar)
//     {
//         ROS_ERROR("lidar loop back, clear buffer");

//         return;
//         // lidar_buffer.shrink_to_fit();
//     }

//     last_timestamp_lidar = msg->header.stamp.toSec();    


//     PointCloudXYZI::Ptr  ptr(new PointCloudXYZI(10000,1));
//     p_pre->process(msg, ptr); 
//     if (ptr->points.size() > 0)
//     {
//         lidar_buffer.emplace_back(ptr);
//         time_buffer.emplace_back(msg->header.stamp.toSec());
//     }
// }


void li_initialization::imu_cbk(const std::shared_ptr<const sensor_msgs::msg::Imu> msg_in) 
{
    sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));

    double timestamp = rclcpp::Time(msg->header.stamp).seconds();
    // printf("imu time %.4f\n", timestamp);

    if (timestamp < last_timestamp_imu)
    {
        std::cout << "imu loop back, clear buffer" << std::endl;
        imu_deque.shrink_to_fit();
        return;
    }
    imu_deque.emplace_back(msg);
    last_timestamp_imu = timestamp;

    // if (!is_first_frame){
    //     state_output pre_state_out = kf_output.x_;
      

    //     state_output now_state_out;
    //     double dt = timestamp - time_predict_last_const;
    //     kf_output.predict(dt, input_in, pre_state_out, now_state_out); // 相当于调用了一个函数，不涉及 kf_output 内成员状态变化

    //     nav_msgs::Odometry odom_imufreq;
    //     {  
    //         odom_imufreq.header.frame_id = "camera_init";
    //         odom_imufreq.child_frame_id = "body_pridict";
            
    //         odom_imufreq.header.stamp = msg->header.stamp;

    //         odom_imufreq.pose.pose.position.x = now_state_out.pos(0);
    //         odom_imufreq.pose.pose.position.y = now_state_out.pos(1);
    //         odom_imufreq.pose.pose.position.z = now_state_out.pos(2);
    //         Eigen::Quaterniond q(now_state_out.rot);
    //         odom_imufreq.pose.pose.orientation.x = q.coeffs()[0];
    //         odom_imufreq.pose.pose.orientation.y = q.coeffs()[1];
    //         odom_imufreq.pose.pose.orientation.z = q.coeffs()[2];
    //         odom_imufreq.pose.pose.orientation.w = q.coeffs()[3];

    //         pubOdom_imufreq.publish(odom_imufreq);
    //     }

    //     {
    //         static tf::TransformBroadcaster br;
    //         tf::Transform                   transform;
    //         tf::Quaternion                  q;
    //         transform.setOrigin(tf::Vector3(odom_imufreq.pose.pose.position.x, \
    //                                         odom_imufreq.pose.pose.position.y, \
    //                                         odom_imufreq.pose.pose.position.z));
    //         q.setW(odom_imufreq.pose.pose.orientation.w);
    //         q.setX(odom_imufreq.pose.pose.orientation.x);
    //         q.setY(odom_imufreq.pose.pose.orientation.y);
    //         q.setZ(odom_imufreq.pose.pose.orientation.z);
    //         transform.setRotation( q );
    //         br.sendTransform( tf::StampedTransform( transform, odom_imufreq.header.stamp, "camera_init", "body_pridict") );
    //     }
    // }
}


void li_initialization::reset_cbk(const std::shared_ptr<const std_msgs::msg::Bool> msg){
    std::cout << "Received bool: " << msg->data << std::endl;
    // exit(0);
    reset_flag = true;
}

void li_initialization::initialpose_callback(const std::shared_ptr<const geometry_msgs::msg::PoseWithCovarianceStamped> input)
{
//   Eigen::Quaterniond q;
//   q.x() = input->pose.pose.orientation.x;
//   q.y() = input->pose.pose.orientation.y;
//   q.z() = input->pose.pose.orientation.z;
//   q.w() = input->pose.pose.orientation.w;
//   manual_reloc_pose.block<3, 3>(0, 0) =  q.toRotationMatrix();
//   manual_reloc_pose(0, 3) = input->pose.pose.position.x;
//   manual_reloc_pose(1, 3) = input->pose.pose.position.y;
//   manual_reloc_pose(2, 3) = ManualReloPose_z_;
//   manual_reloc_pose.block<1, 4>(3, 0) << 0, 0, 0, 1;
//   manual_reloc_flag = true;
}


bool li_initialization::sync_packages(MeasureGroup &meas, bool imu_need_init)
{
    if (lidar_buffer.empty() || imu_deque.empty())
    {
        return false;
    }

    // std::cout << "lidar_buffer.size(): " << lidar_buffer.size() << std::endl;
    // std::cout << "imu_deque.size(): " << imu_deque.size() << std::endl;
    /*** push a lidar scan ***/
    // 填充 meas.lidar 、meas.lidar_beg_time、meas.lidar_last_time 
    // lidar_pushed主要作用是，保证在 last_timestamp_imu > lidar_end_time条件满足之前，不（重复）更新雷达数据，保证初始化时 imu最新时间戳大于雷达最后时间戳
    // 这个等待一般就几个imu帧的时间
    if(!lidar_pushed)
    {
        meas.lidar = lidar_buffer.back();
        meas.lidar_beg_time = time_buffer.back();
        
        // 确认 lidar_end_time curvature， 在这里是起始点到当前点的 offset time 的意思，遍历确认最后的时间戳
        double end_time = meas.lidar->points.back().curvature;
        for (auto pt: meas.lidar->points)
        {
            if (pt.curvature > end_time)
            {
                end_time = pt.curvature;
            }
        }
        lidar_end_time = meas.lidar_beg_time + end_time / double(1000);
        // cout << "check time lidar:" << end_time << endl;
        meas.lidar_last_time = lidar_end_time;
            
        lidar_pushed = true;
    }
    

    // 等待 imu_last_timestamp > lidar_end_time条件满足
    // 这个等待一般就几个imu帧的时间
    // 判断最后一帧imu时间戳务必大于后一帧雷达点时间戳，后面插值对齐需要这个条件
    double imu_last_timestamp = rclcpp::Time(imu_deque.back()->header.stamp).seconds();
    if (imu_last_timestamp < lidar_end_time)
    {
        return false;
    }

    // printf("imu_last_timestamp: %.4f\n", imu_last_timestamp);
    // printf("lidar_end_time: %.4f\n", lidar_end_time);
    // printf("meas.lidar->points.front().curvature: %.4f\n", meas.lidar->points.front().curvature);
    // printf("meas.lidar->points.back().curvature: %.4f\n", meas.lidar->points.back().curvature);
    // printf("time_buffer.back(): %.4f\n", time_buffer.front());
    
    /*** push imu data, and pop from imu buffer ***/
    // imu_need_init 在 p_imu 实例化时确定为 true，代表需要imu初始化。 调用 p_imu->Process 后会变为 false
    // 这个只在 imu 初始化时才会更新 meas.imu，这个初始化会进行若干次
    if (imu_need_init)
    {
        // 一直遍历到 当前的imu数据时间戳刚好大于最后一个雷达点时间戳 
        // 把时间戳小于 lidar_end_time 之前的 imu 数据放到 meas.imu 并删除
        while ((!imu_deque.empty()))
        {
            double imu_time = rclcpp::Time(imu_deque.front()->header.stamp).seconds();
            if(imu_time > lidar_end_time) break;
            meas.imu.emplace_back(imu_deque.front());
            imu_deque.pop_front();
        }
        // // printf("imu_time: %.4f", imu_time);
        // printf("imu_deque.front()->header.stamp.toSec(): %.4f", rclcpp::Time(imu_deque.front()->header.stamp).seconds());
        // printf("lidar_end_time: %.4f", lidar_end_time);
    }

    lidar_buffer.clear(); // 从激光雷达数据队列里删除处理过的数据
    time_buffer.clear(); // 从time_buffer数据队列里删除处理过的数据
    lidar_pushed = false;
    return true;
}


void li_initialization::reset(){
    lidar_buffer.clear();
    time_buffer.clear();
    imu_deque.clear();
    last_timestamp_lidar = -1.0, last_timestamp_imu = -1.0;
    lidar_pushed = false;
}