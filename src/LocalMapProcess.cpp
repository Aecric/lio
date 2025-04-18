#include "LocalMapProcess.h"


LocalMapProcess::LocalMapProcess(){
    ivox_map_set.reset(new IVoxType(ivox_options_));
    ivox_map_get.reset(new IVoxType(ivox_options_));
    Global_map_load.reset(new PointCloudXYZI());
}

void LocalMapProcess::run(){
    std::shared_ptr<IVoxType> ivox_map = std::make_shared<IVoxType>(ivox_options_);
    std::shared_ptr<IVoxType> ivox_map_copy = std::make_shared<IVoxType>(ivox_options_);
    pcl::PointCloud<PointType> current_points;
    PointType current_pose;
    PointType last_pose;
    double current_timestamp;

    if(pure_loc){
        // load map
        if (pcl::io::loadPCDFile(root_dir+map_path, *Global_map_load) == 0) {
            std::cout << "[LocalMapProcess] Successfully loaded Global_map: " << map_path << std::endl;
            std::cout << "[LocalMapProcess] PointCloud size: " << Global_map_load->points.size() << std::endl;
        } else {
            std::cerr << "[LocalMapProcess] Failed to load Global_map PCD file: " << map_path << std::endl;
            exit(0);
        }

        Global_map_bool_vector.resize(Global_map_load->size(), false);
        pure_loc_flag = true;
    }
    while(true){
        // 等待初始 ivox 数据到来
        if(!init_ivox_flag){
            std::unique_lock<std::mutex> lock(mMutex_ivox_set);
            if(ivox_map_set_flag){
                // *ivox_map = *ivox_map_set;
                ivox_map.swap(ivox_map_set);
                ivox_map_set.reset(new IVoxType(ivox_options_));
                ivox_map_set_flag = false;
                init_ivox_flag = true;
            }
            lock.unlock();

            usleep(10000);
            continue;
        }



        // 获取定位帧数据
        {
            bool _flag = false;
            std::unique_lock<std::mutex> lock(mMutex_points);
            if(current_points_set_flag){
                current_points = current_points_set;
                current_pose = current_pose_set;
                current_timestamp = current_pose_timestamp;
                current_points_set.clear();
                _flag = true;
                current_points_set_flag = false;
            }
            lock.unlock();

            if(!_flag){
                usleep(5000);
                continue;
            }
        }

        double distance = std::sqrt(
            std::pow(current_pose.x - last_pose.x, 2) +
            std::pow(current_pose.y - last_pose.y, 2) +
            std::pow(current_pose.z - last_pose.z, 2)
        );
        
        // 构建、管理、打包 ivox 地图
        double clc_timestamp_increase_map_start = omp_get_wtime();
        pcl::PointCloud<PointType> increase_map_;
        increase_map_.points.clear();
        increase_map_.points.reserve(50000);
        if(pure_loc_flag && distance > 5){
            // ResetLocalMap
            for (int i = 0; i < Global_map_load->points.size(); i++){
                if (abs(Global_map_load->points[i].x - current_pose.x) < 100 &&
                    abs(Global_map_load->points[i].y - current_pose.y) < 100 &&
                    abs(Global_map_load->points[i].z - current_pose.z) < 100){
                    if(!Global_map_bool_vector[i]){
                        PointType p;
                        p.x = Global_map_load->points[i].x;
                        p.y = Global_map_load->points[i].y;
                        p.z = Global_map_load->points[i].z;
                        p.intensity = Global_map_load->points[i].intensity;
                        Global_map_bool_vector[i] = true;
                        increase_map_.points.emplace_back(p);
                    }
                }else{
                    Global_map_bool_vector[i] = false;
                    continue;
                }

            }
            std::cout << "[LocalMapProcess] increase_map_.size(): "<< increase_map_.size() << std::endl;
            ivox_map->AddPoints(increase_map_.points);
        }else{
            ivox_map->AddPoints(current_points.points);
        }
        // std::cout << "increase_map time: " << omp_get_wtime() - clc_timestamp_increase_map_start << std::endl;
        // exit(0);

        ivox_map->CutKeys(current_pose, 50.0);
        std::cout << "[LocalMapProcess] current_points.size(): " << current_points.size() << std::endl;
        current_points.clear();

        double clc_timestamp_get_ivox_start = omp_get_wtime();

        // 深拷贝耗时，提前深拷贝
        *ivox_map_copy = *ivox_map;
        {
            std::unique_lock<std::mutex> lock(mMutex_ivox_get);
            ivox_map_get_flag = true;
            ivox_map_get.swap(ivox_map);
            lock.unlock();
        }
        ivox_map.swap(ivox_map_copy);

        last_pose = current_pose;

        std::cout << "[LocalMapProcess] get_ivox func time: " << omp_get_wtime() - clc_timestamp_get_ivox_start << std::endl;
        std::cout << "[LocalMapProcess] ivox_map->NumValidGrids(): " << ivox_map->NumValidGrids() << std::endl;
        
        

        // 以 ivox 内点云时间戳分割点云关键帧




        // 防止 cpu 占用过高
        usleep(1000);
    }
}







void LocalMapProcess::Reset(){
    ivox_map_set.reset(new IVoxType(ivox_options_));
    ivox_map_get.reset(new IVoxType(ivox_options_));
    ivox_map_get_flag = false;
    ivox_map_set_flag = false;
    current_points_set_flag = false;
    init_ivox_flag = false;
}












