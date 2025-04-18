#ifndef LocalMapProcess_H
#define LocalMapProcess_H

#include "common_lib.h"
#include "parameters.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
// #include <ikd-Tree/ikd_Tree.h>
#include <pcl/io/pcd_io.h>
#include <unordered_set>
#include "parameters.h"


class LocalMapProcess
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // ros::Publisher pubGlobalmap;

    LocalMapProcess();
    // LocalMapProcess() = default;
    ~LocalMapProcess() = default;
    
    std::shared_ptr<IVoxType> ivox_map_get = nullptr; 
    bool ivox_map_get_flag = false;

    std::shared_ptr<IVoxType> ivox_map_set = nullptr; 
    bool ivox_map_set_flag = false;

    pcl::PointCloud<PointType> current_points_set;
    PointType current_pose_set;
    double current_pose_timestamp;
    bool current_points_set_flag = false;

    bool init_ivox_flag = false;

    // pcl::PointCloud<PointType> Global_map_load;
    PointCloudXYZI::Ptr Global_map_load;
    std::vector<bool> Global_map_bool_vector;
    bool pure_loc_flag = false;


    void run();

    void Reset();

    void set_ivox(IVoxType iv){
        std::unique_lock<std::mutex> lock(mMutex_ivox_set);
        *ivox_map_set = iv; // 为了不影响原数据，这里直接进行深拷贝
        ivox_map_set_flag = true;
        lock.unlock();
        std::cout << "[LocalMapProcess] set_ivox: " << ivox_map_set->NumValidGrids() << std::endl;
    };

    void get_ivox(std::shared_ptr<IVoxType> iv){
        std::unique_lock<std::mutex> lock(mMutex_ivox_get);
        if(ivox_map_get_flag){
            // double clc_timestamp_start = omp_get_wtime();
            // *iv = *ivox_map_get; // 深拷贝耗时。如果目标对象很大，或包含大量动态分配的元素（如 std::vector、std::unordered_map），分配内存和拷贝数据的过程会非常耗时。
            iv.swap(ivox_map_get);
            // std::cout << "iv from ivox_map_get use time: " << omp_get_wtime() - clc_timestamp_start << std::endl;

            ivox_map_get_flag = false;
        }
        lock.unlock();
    };

    void set_currentpts(pcl::PointCloud<PointType> pts, PointType pose, double t){
        std::unique_lock<std::mutex> lock(mMutex_points);
        current_points_set += pts;
        current_pose_set = pose;
        current_pose_timestamp = t;
        current_points_set_flag = true;
        lock.unlock();
        // std::cout << "set_currentpts: " << current_points_set.size() << std::endl;
    };

    void set_map(pcl::PointCloud<PointType> mp){
        double clc_timestamp_start = omp_get_wtime();
        std::unique_lock<std::mutex> lock(mMutex_map);
        pure_loc_flag = true;
        *Global_map_load = mp;
        lock.unlock();
        // 初始化 Global_map_load.size() 个元素，值为 false
        Global_map_bool_vector.resize(Global_map_load->size(), false);

        std::cout << "set_map use time: " << omp_get_wtime() - clc_timestamp_start << std::endl;
    };

    std::mutex mMutex_ivox_set;
    std::mutex mMutex_ivox_get;
    std::mutex mMutex_points;
    std::mutex mMutex_map;
private:
};














#endif