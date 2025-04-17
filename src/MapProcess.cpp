#include "MapProcess.h"



void MapProcess::run(){
    // ivox_map = std::make_shared<IVoxType>(ivox_options_);

    // 设置固定核心
    {
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(1, &cpuset);
        if(0 != pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset)){
            std::cout<<"Error: Set affinity failed in MapProcess::run."<<std::endl;
            exit(-2);
        }
    }
    while(true){
        usleep(1000);

        std::vector<IVoxType> ivox_maps_;
        std::unique_lock<std::mutex> lock(mMutexGet);
        ivox_maps_ = ivox_maps;
        ivox_maps.clear();
        lock.unlock();

        if(ivox_maps_.empty()){
            continue;
        }

        std::cout << "ivox_maps_.size(): " << ivox_maps_.size() << std::endl;
        // exit(0);
        // 以ivox内点云时间戳分割点云关键帧

    }
}


















