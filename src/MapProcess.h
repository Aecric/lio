#ifndef MapProcess_H
#define MapProcess_H

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


class MapProcess
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    MapProcess() = default;
    ~MapProcess() = default;
    
    std::vector<IVoxType> ivox_maps;

    void run();

    void set_ivox(IVoxType iv){
        IVoxType iv_map = iv;
        std::unique_lock<std::mutex> lock(mMutexGet);
        ivox_maps.emplace_back(iv_map);
        std::cout << "ivox_maps.size(): " << ivox_maps.size() << std::endl;
        lock.unlock();
    };

    void Reset();
    void Process(const MeasureGroup &meas);

    std::mutex mMutexGet;
private:
};














#endif