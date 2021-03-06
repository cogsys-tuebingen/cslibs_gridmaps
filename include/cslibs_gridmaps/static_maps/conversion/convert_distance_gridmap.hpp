#ifndef CSLIBS_GRIDMAPS_CONVERT_DISTANCE_GRIDMAP_HPP
#define CSLIBS_GRIDMAPS_CONVERT_DISTANCE_GRIDMAP_HPP

#include <cslibs_gridmaps/static_maps/distance_gridmap.h>
#include <cslibs_gridmaps/static_maps/algorithms/distance_transform.hpp>

#include <tf/tf.h>
#include <nav_msgs/OccupancyGrid.h>

namespace cslibs_gridmaps {
namespace static_maps {
namespace conversion {
template <typename Tp, typename T, typename AllocatorT = std::allocator<T>>
inline void from(const nav_msgs::OccupancyGrid &src,
                 typename DistanceGridmap<Tp, T, AllocatorT>::Ptr &dst,
                 const double threshold = 1.0,
                 const T maximum_distance = 2.0)
{
    assert(threshold <= 1.0);
    assert(threshold >= 0.0);

    std::vector<int8_t> data = src.data;
//    for(int8_t &i : data) {
//        i = i == -1 ? 100 : i;
//    }

    cslibs_math_2d::Pose2<Tp> origin(src.info.origin.position.x,
                                      src.info.origin.position.y,
                                      tf::getYaw(src.info.origin.orientation));

    dst.reset(new DistanceGridmap<Tp, T, AllocatorT>(origin,
                                                     src.info.resolution,
                                                     maximum_distance,
                                                     src.info.height,
                                                     src.info.width,
                                                     maximum_distance));

    algorithms::DistanceTransform<Tp,T,int8_t,AllocatorT> distance_transform(
                src.info.resolution,
                maximum_distance,
                static_cast<int8_t>(threshold * 100));
    distance_transform.apply(data,
                             dst->getWidth(),
                             dst->getData());
}

template <typename Tp, typename T, typename AllocatorT = std::allocator<T>>
inline void from(DistanceGridmap<Tp, T, AllocatorT> &src,
                 nav_msgs::OccupancyGrid::Ptr &dst)
{
    dst.reset(new nav_msgs::OccupancyGrid);
    dst->info.resolution         = static_cast<float>(src.getResolution());
    dst->info.height             = static_cast<unsigned int>(src.getHeight());
    dst->info.width              = static_cast<unsigned int>(src.getWidth());
    dst->info.origin.position.x  = src.getOrigin().tx();
    dst->info.origin.position.y  = src.getOrigin().ty();
    dst->info.origin.orientation = tf::createQuaternionMsgFromYaw(src.getOrigin().yaw());
    dst->info.map_load_time = dst->header.stamp;
    dst->data.resize(src.getData().size());
    std::transform(src.getData().begin(), src.getData().end(),
                   dst->data.begin(),
                   [](const T p){return 100.0 * p;});
}
}
}
}

#endif // CSLIBS_GRIDMAPS_CONVERT_DISTANCE_GRIDMAP_HPP
