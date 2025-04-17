//
// Created by xiang on 2021/9/16.
//

#ifndef FASTER_LIO_IVOX3D_H
#define FASTER_LIO_IVOX3D_H

// #include <execution>
#include <list>
#include <thread>
#include <unordered_map>

#include "tsl/robin_map.h"
#include "eigen_types.h"
#include "ivox3d_node.hpp"

extern double serch_time;
namespace faster_lio {

enum class IVoxNodeType {
    DEFAULT,  // linear ivox
    PHC,      // phc ivox
};

/// traits for NodeType
template <IVoxNodeType node_type, typename PointT, int dim>
struct IVoxNodeTypeTraits {};

template <typename PointT, int dim>
struct IVoxNodeTypeTraits<IVoxNodeType::DEFAULT, PointT, dim> {
    using NodeType = IVoxNode<PointT, dim>;
};

template <typename PointT, int dim>
struct IVoxNodeTypeTraits<IVoxNodeType::PHC, PointT, dim> {
    using NodeType = IVoxNodePhc<PointT, dim>;
};

template <int dim = 3, IVoxNodeType node_type = IVoxNodeType::DEFAULT, typename PointType = pcl::PointXYZ>
class IVox {
   public:
    using KeyType = Eigen::Matrix<int, dim, 1>;
    using PtType = Eigen::Matrix<float, dim, 1>;
    using NodeType = typename IVoxNodeTypeTraits<node_type, PointType, dim>::NodeType;
    using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;
    using DistPoint = typename NodeType::DistPoint;

    enum class NearbyType {
        CENTER,  // center only
        NEARBY6,
        NEARBY18,
        NEARBY26,
    };

    struct Options {
        float resolution_ = 0.2;                        // ivox resolution
        float inv_resolution_ = 10.0;                   // inverse resolution
        NearbyType nearby_type_ = NearbyType::NEARBY6;  // nearby range
        std::size_t capacity_ = 1000000;                // capacity
        float insert_resolution_ = 0.1;                        // ivox insert_resolution_
    };

    /**
     * constructor
     * @param options  ivox options
     */
    explicit IVox(Options options) : options_(options) {
        options_.inv_resolution_ = 1.0 / options_.resolution_;
        GenerateNearbyGrids();
    }

    /**
     * add points
     * @param points_to_add
     */
    void AddPoints(const PointVector& points_to_add);

    /// get nn
    bool GetClosestPoint(const PointType& pt, PointType& closest_pt);

    /// get nn with condition
    bool GetClosestPoint(const PointType& pt, PointVector& closest_pt, int max_num = 5, double max_range = 5.0);

    /// get nn in cloud
    bool GetClosestPoint(const PointVector& cloud, PointVector& closest_cloud);

    /// get number of points
    size_t NumPoints() const;

    /// get number of valid grids
    size_t NumValidGrids() const;

    /// get statistics of the points
    // std::vector<float> StatGridPoints() const;

    // .find(dkey) 返回的是一个迭代器，而不是直接返回 value。
    // 这个迭代器指向的是 std::unordered_map 中的一个 std::pair<const KeyType, NodeType>，也就是说，它指向的是一个包含 key 和 value 的键值对。
    // std::unordered_map<KeyType, typename std::list<std::pair<KeyType, NodeType>>::iterator, hash_vec<dim>>
    //     grids_map_;   
    std::unordered_map<KeyType, NodeType, hash_vec<dim>>
        grids_map_;   
    KeyType Pos2Grid(const PtType& pt) const;
    KeyType Pos2Grid_(const PtType& pt, const double &defined_res) const;

    std::vector<PointType> GetAllPoints(){
        std::vector<PointType> all_points;
        all_points.reserve(1000000);
        // 遍历 unordered_map 的每个键值对
        for (const auto& pair : grids_map_) {
            // const KeyType& key = pair.first;    // 获取 key
            const NodeType& value = pair.second; // 获取对应的 value
            
            // 收集点云
            all_points.insert(all_points.end(), value.points_.begin(), value.points_.end());
        }
        return all_points;
    };

    void CutKeys(PointType pose, float far_){
        auto key_p = Pos2Grid(ToEigen<float, dim>(pose));
        float boundary = far_/options_.resolution_;
        // std::cout << "boundary: " << boundary << std::endl;
        // std::cout << "key_p: " << key_p << std::endl;
        // std::cout << "cut before NumValidGrids: " << NumValidGrids() << std::endl;

        // 遍历 unordered_map 的每个键值对，并删除过远的key
        for (auto it = grids_map_.begin(); it != grids_map_.end(); ) {

            const KeyType& key = it->first;    // 获取 key
            if(abs(key[0]-key_p[0]) > boundary ||
            abs(key[1]-key_p[1]) > boundary ||
            abs(key[2]-key_p[2]) > boundary){
                // 删除当前迭代器指向的元素，并将迭代器移动到下一个位置
                it = grids_map_.erase(it);
            } else {
                // 如果不删除，则继续遍历下一个元素
                ++it;
            }
        }
        // std::cout << "cut after NumValidGrids: " << NumValidGrids() << std::endl;
    };

   private:
    /// generate the nearby grids according to the given options
    void GenerateNearbyGrids();

    /// position to grid
    // KeyType Pos2Grid(const PtType& pt) const;

    Options options_;
    // std::unordered_map<KeyType, typename std::list<std::pair<KeyType, NodeType>>::iterator, hash_vec<dim>>
        // grids_map_;                                        // voxel hash map
    // std::list<std::pair<KeyType, NodeType>> grids_cache_;  // voxel cache
    std::vector<KeyType> nearby_grids_;                    // nearbys
};

template <int dim, IVoxNodeType node_type, typename PointType>
bool IVox<dim, node_type, PointType>::GetClosestPoint(const PointType& pt, PointType& closest_pt) {
    std::vector<DistPoint> candidates;
    auto key = Pos2Grid(ToEigen<float, dim>(pt));
    std::for_each(nearby_grids_.begin(), nearby_grids_.end(), [&key, &candidates, &pt, this](const KeyType& delta) {
        auto dkey = key + delta;
        auto iter = grids_map_.find(dkey);
        if (iter != grids_map_.end()) {
            DistPoint dist_point;
            // bool found = iter->second->second.NNPoint(pt, dist_point);
            bool found = iter->second.NNPoint(pt, dist_point);
            if (found) {
                candidates.emplace_back(dist_point);
            }
        }
    });

    if (candidates.empty()) {
        return false;
    }

    auto iter = std::min_element(candidates.begin(), candidates.end());
    closest_pt = iter->Get();
    return true;
}

template <int dim, IVoxNodeType node_type, typename PointType>
bool IVox<dim, node_type, PointType>::GetClosestPoint(const PointType& pt, PointVector& closest_pt, int max_num,
                                                      double max_range) {
    closest_pt.clear();
    std::vector<DistPoint> candidates;
    candidates.reserve(max_num * nearby_grids_.size());

    auto key = Pos2Grid(ToEigen<float, dim>(pt));
    // std::cout << "max_range: " << max_range << std::endl;

// #define INNER_TIMER
#ifdef INNER_TIMER
    static std::unordered_map<std::string, std::vector<int64_t>> stats;
    if (stats.empty()) {
        stats["knn"] = std::vector<int64_t>();
        stats["nth"] = std::vector<int64_t>();
    }
#endif
    int a_count = 0;
    for (const KeyType& delta : nearby_grids_) {
        auto dkey = key + delta;
        auto iter = grids_map_.find(dkey);
        if (iter != grids_map_.end()) {
#ifdef INNER_TIMER
            auto t1 = std::chrono::high_resolution_clock::now();
#endif
	double serch_start = omp_get_wtime();
    // iter->second 是一个指向 std::list<std::pair<KeyType, NodeType>> 中某个元素的迭代器，这个元素是一个 std::pair<KeyType, NodeType>
    // iter->second 是指向 std::pair<KeyType, NodeType> 的迭代器。
    // iter->second->first 是 KeyType，即这个 pair 中的键。
    // iter->second->second 是 NodeType，即这个 pair 中的值。
            // auto tmp = iter->second->second.KNNPointByCondition(candidates, pt, max_num, max_range);
            auto tmp = iter->second.KNNPointByCondition(candidates, pt, max_num, max_range);
            a_count+=tmp;
	serch_time += omp_get_wtime() - serch_start;
        
#ifdef INNER_TIMER
            auto t2 = std::chrono::high_resolution_clock::now();
            auto knn = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count();
            stats["knn"].emplace_back(knn);
#endif
        }
    }
    // std::cout << "a_count: " << a_count << std::endl;


    if (candidates.empty()) {
        return false;
    }

#ifdef INNER_TIMER
    auto t1 = std::chrono::high_resolution_clock::now();
#endif
    if (candidates.size() <= max_num) {
        // 反正最近点小于 max_num 也不会被使用...
        return false;
    } else {
        std::nth_element(candidates.begin(), candidates.begin() + max_num - 1, candidates.end());
        candidates.resize(max_num);
    }
    // 不知道这一步用来干啥
    // std::nth_element(candidates.begin(), candidates.begin(), candidates.end());


#ifdef INNER_TIMER
    auto t2 = std::chrono::high_resolution_clock::now();
    auto nth = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count();
    stats["nth"].emplace_back(nth);

    constexpr int STAT_PERIOD = 100000;
    if (!stats["nth"].empty() && stats["nth"].size() % STAT_PERIOD == 0) {
        for (auto& it : stats) {
            const std::string& key = it.first;
            std::vector<int64_t>& stat = it.second;
            int64_t sum_ = std::accumulate(stat.begin(), stat.end(), 0);
            int64_t num_ = stat.size();
            stat.clear();
            std::cout << "inner_" << key << "(ns): sum=" << sum_ << " num=" << num_ << " ave=" << 1.0 * sum_ / num_
                      << " ave*n=" << 1.0 * sum_ / STAT_PERIOD << std::endl;
        }
    }
#endif

    for (auto& it : candidates) {
        closest_pt.emplace_back(it.Get());
    }
    return closest_pt.empty() == false;
}

template <int dim, IVoxNodeType node_type, typename PointType>
size_t IVox<dim, node_type, PointType>::NumValidGrids() const {
    return grids_map_.size();
}

template <int dim, IVoxNodeType node_type, typename PointType>
void IVox<dim, node_type, PointType>::GenerateNearbyGrids() {
    if (options_.nearby_type_ == NearbyType::CENTER) {
        nearby_grids_.emplace_back(KeyType::Zero());
    } else if (options_.nearby_type_ == NearbyType::NEARBY6) {
        nearby_grids_ = {KeyType(0, 0, 0),  KeyType(-1, 0, 0), KeyType(1, 0, 0), KeyType(0, 1, 0),
                         KeyType(0, -1, 0), KeyType(0, 0, -1), KeyType(0, 0, 1)};
    } else if (options_.nearby_type_ == NearbyType::NEARBY18) {
        nearby_grids_ = {KeyType(0, 0, 0),  KeyType(-1, 0, 0), KeyType(1, 0, 0),   KeyType(0, 1, 0),
                         KeyType(0, -1, 0), KeyType(0, 0, -1), KeyType(0, 0, 1),   KeyType(1, 1, 0),
                         KeyType(-1, 1, 0), KeyType(1, -1, 0), KeyType(-1, -1, 0), KeyType(1, 0, 1),
                         KeyType(-1, 0, 1), KeyType(1, 0, -1), KeyType(-1, 0, -1), KeyType(0, 1, 1),
                         KeyType(0, -1, 1), KeyType(0, 1, -1), KeyType(0, -1, -1)};
    } else if (options_.nearby_type_ == NearbyType::NEARBY26) {
        nearby_grids_ = {KeyType(0, 0, 0),   KeyType(-1, 0, 0),  KeyType(1, 0, 0),   KeyType(0, 1, 0),
                         KeyType(0, -1, 0),  KeyType(0, 0, -1),  KeyType(0, 0, 1),   KeyType(1, 1, 0),
                         KeyType(-1, 1, 0),  KeyType(1, -1, 0),  KeyType(-1, -1, 0), KeyType(1, 0, 1),
                         KeyType(-1, 0, 1),  KeyType(1, 0, -1),  KeyType(-1, 0, -1), KeyType(0, 1, 1),
                         KeyType(0, -1, 1),  KeyType(0, 1, -1),  KeyType(0, -1, -1), KeyType(1, 1, 1),
                         KeyType(-1, 1, 1),  KeyType(1, -1, 1),  KeyType(1, 1, -1),  KeyType(-1, -1, 1),
                         KeyType(-1, 1, -1), KeyType(1, -1, -1), KeyType(-1, -1, -1)};
    } else {
        // LOG(ERROR) << "Unknown nearby_type!";
    }
}

template <int dim, IVoxNodeType node_type, typename PointType>
bool IVox<dim, node_type, PointType>::GetClosestPoint(const PointVector& cloud, PointVector& closest_cloud) {
    std::vector<size_t> index(cloud.size());
    
    closest_cloud.resize(cloud.size());

    for (int i = 0; i < cloud.size(); ++i) {
        PointType pt;
        if (GetClosestPoint(cloud[i], pt)) {
            closest_cloud[i] = pt;
        } else {
            closest_cloud[i] = PointType();
        }
    };
    return true;
}

template <int dim, IVoxNodeType node_type, typename PointType>
void IVox<dim, node_type, PointType>::AddPoints(const PointVector& points_to_add) {
    int count = 0;
    for(size_t i = 0; i<points_to_add.size(); i++) {
        auto key = Pos2Grid(Eigen::Matrix<float, dim, 1>(points_to_add[i].x, points_to_add[i].y, points_to_add[i].z));
        auto iter = grids_map_.find(key);
        if (iter == grids_map_.end()) { // 表示哈希表里没有这个key
            PointType center;
            center.getVector3fMap() = key.template cast<float>() * options_.resolution_;

            // grids_cache_.push_front({key, NodeType(center, options_.resolution_)});
            // grids_map_.insert({key, grids_cache_.begin()});

            // grids_cache_.front().second.InsertPoint(points_to_add[i]);

            // if (grids_map_.size() >= options_.capacity_) {
            //     grids_map_.erase(grids_cache_.back().first);
            //     grids_cache_.pop_back();
            // }
            grids_map_.insert({key, NodeType(center, options_.resolution_)});
            grids_map_[key].InsertPoint(points_to_add[i]);
            count++;
        } else {
        //     // 遍历 key 内点云，若存在已有距离太近的点，则不插入
        //     std::size_t count_points = iter->second->second.Size();
        //     bool filter_flag = true;
        //     for(int m = 0; m < count_points; m++){
        //         if(abs(iter->second->second.GetPoint(m).x - points_to_add[i].x) < 0.1 &&
        //            abs(iter->second->second.GetPoint(m).y - points_to_add[i].y) < 0.1 &&
        //            abs(iter->second->second.GetPoint(m).z - points_to_add[i].z) < 0.1){
        //             filter_flag = false;
        //             break;
        //         }
        //         // else{
        //         //     double d_ = distance2(iter->second->second.GetPoint(m), points_to_add[i]);
        //         //     std::cout << "d_: " << d_ << std::endl;
        //         // }
        //     }

        //     if(filter_flag){ // 控制key内点云规模  && count_points < 500
        //     iter->second->second.InsertPoint(points_to_add[i]);
        //     // std::size_t count_points = iter->second->second.Size();
        //     // std::cout << "count_points: " << count_points << std::endl;
        //     // if(count_points > 1000){exit(0);}
        //     // 将当前键值移动到list头
        //     grids_cache_.splice(grids_cache_.begin(), grids_cache_, iter->second);
        //     grids_map_[key] = grids_cache_.begin();
        //     }
            // 遍历 key 内点云，若存在已有距离太近的点，则不插入
            std::size_t count_points = iter->second.Size();
            bool filter_flag = true;
            for(int m = 0; m < count_points; m++){
                if(abs(iter->second.GetPoint(m).x - points_to_add[i].x) < options_.insert_resolution_ &&
                   abs(iter->second.GetPoint(m).y - points_to_add[i].y) < options_.insert_resolution_ &&
                   abs(iter->second.GetPoint(m).z - points_to_add[i].z) < options_.insert_resolution_){
                    filter_flag = false;
                    break;
                }
            }

            if(filter_flag){ // 控制key内点云规模  && count_points < 500
            iter->second.InsertPoint(points_to_add[i]);
            // grids_map_[key] = grids_cache_.begin();
            count++;
            }
        }
    }
    // std::cout << "AddPoints count: " << count << std::endl;
}

template <int dim, IVoxNodeType node_type, typename PointType>
Eigen::Matrix<int, dim, 1> IVox<dim, node_type, PointType>::Pos2Grid(const IVox::PtType& pt) const {
    return (pt * options_.inv_resolution_).array().floor().template cast<int>();
}

template <int dim, IVoxNodeType node_type, typename PointType>
Eigen::Matrix<int, dim, 1> IVox<dim, node_type, PointType>::Pos2Grid_(const IVox::PtType& pt, const double &defined_res) const {
    return (pt / defined_res).array().floor().template cast<int>();
}


}  // namespace faster_lio

#endif
