#include "laserMapping.h"
#include <iostream>
#include <Eigen/Eigen>
#include <string>
#include <unistd.h>
#include "parameters.h"

int main(int argc, char** argv)
{
    std::cout << "root_dir: " << root_dir << std::endl;

    // 设置固定核心
    {
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(1, &cpuset);
        if(0 != pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset)){
            std::cout<<"Error: Set affinity failed in main."<<std::endl;
            exit(-2);
        }
    }

    YAML::Node config = YAML::LoadFile(root_dir + "config/avia.yaml");
    readParameters(config);

    rclcpp::init(0, nullptr);
    // 创建 ROS2 节点
    rclcpp::Node::SharedPtr nh_ = std::make_shared<rclcpp::Node>("laser_mapping");

    laserMapping *lio = new laserMapping(nh_);
    std::thread laserMapping_process_thread = std::thread(&laserMapping::run, lio);
    // 设置固定核心
    {
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(2, &cpuset);
        if(0 != pthread_setaffinity_np(laserMapping_process_thread.native_handle(), sizeof(cpu_set_t), &cpuset)){
            std::cout<<"Error: Set affinity failed in laserMapping::run."<<std::endl;
            exit(-2);
        }
    }

    LocalMapProcess *localmap_process_handle_ = new LocalMapProcess();
    lio->setLocalMapHandle(localmap_process_handle_);
    std::thread *localmap_process_thread; 
    if(asyn_locmap){
        localmap_process_thread = new std::thread(&LocalMapProcess::run, localmap_process_handle_);
        // 设置固定核心
        {
            cpu_set_t cpuset;
            CPU_ZERO(&cpuset);
            CPU_SET(2, &cpuset);
            if(0 != pthread_setaffinity_np(localmap_process_thread->native_handle(), sizeof(cpu_set_t), &cpuset)){
                std::cout<<"Error: Set affinity failed in MapProcess::run."<<std::endl;
                exit(-2);
            }
        } 
    }

    rclcpp::spin(nh_);
    rclcpp::shutdown();

    return 0;
}
