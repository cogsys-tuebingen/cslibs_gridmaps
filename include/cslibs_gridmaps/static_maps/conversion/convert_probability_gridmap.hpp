#ifndef CSLIBS_GRIDMAPS_CONVERT_PROBABILITY_GRIDMAP_HPP
#define CSLIBS_GRIDMAPS_CONVERT_PROBABILITY_GRIDMAP_HPP

#include <cslibs_gridmaps/static_maps/probability_gridmap.h>

#include <cslibs_math/common/log_odds.hpp>

#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>

namespace cslibs_gridmaps {
namespace static_maps {
namespace conversion {
template <typename T>
inline int8_t from(const T p)
{
    return p != 0.5 ? static_cast<int8_t>(p * 100.0) : -1;
}

template <typename T>
inline T from(const int8_t p)
{
    return p != -1 ? static_cast<T>(p) * 0.01 : 0.5;
}

template <typename Tp, typename T>
inline void from(const nav_msgs::OccupancyGrid &src,
                 typename ProbabilityGridmap<Tp, T>::Ptr &dst)
{
    cslibs_math_2d::Pose2d<Tp> origin(src.info.origin.position.x,
                                      src.info.origin.position.y,
                                      tf::getYaw(src.info.origin.orientation));

    dst.reset(new ProbabilityGridmap<Tp, T>(origin,
                                            static_cast<double>(src.info.resolution),
                                            src.info.height,
                                            src.info.width));
    std::transform(src.data.begin(), src.data.end(),
                   dst->getData().begin(),
                   [](const int8_t p){return from<T>(p);});
}

template <typename Tp, typename T>
inline void from(const nav_msgs::OccupancyGrid::ConstPtr &src,
                 typename ProbabilityGridmap<Tp, T>::Ptr &dst)
{
   from(*src, dst);
}

template <typename Tp, typename T>
inline void from(const typename ProbabilityGridmap<Tp, T>::Ptr &src,
                 nav_msgs::OccupancyGrid::Ptr &dst)
{
    dst.reset(new nav_msgs::OccupancyGrid);
    dst->info.resolution         = static_cast<float>(src->getResolution());
    dst->info.height             = static_cast<unsigned int>(src->getHeight());
    dst->info.width              = static_cast<unsigned int>(src->getWidth());
    dst->info.origin.position.x  = src->getOrigin().tx();
    dst->info.origin.position.y  = src->getOrigin().ty();
    dst->info.origin.orientation = tf::createQuaternionMsgFromYaw(src->getOrigin().yaw());
    dst->info.map_load_time = dst->header.stamp;
    dst->data.resize(src->getData().size());
    std::transform(src->getData().begin(), src->getData().end(),
                   dst->data.begin(),
                   [](const T p){return from(p);});
}

struct LogOdds {
    template <typename Tp, typename T>
    static inline void to(typename ProbabilityGridmap<Tp, T>::Ptr &src,
                          typename ProbabilityGridmap<Tp, T>::Ptr &dst)
    {
        if(src != dst) {
            dst.reset(new ProbabilityGridmap<Tp, T>(*src));
        }
        std::for_each(dst->getData().begin(),
                      dst->getData().end(),
                      [](T &p){p = cslibs_math::common::LogOdds<T>::to(p);});

    }
    template <typename Tp, typename T>
    static inline void from(typename ProbabilityGridmap<Tp, T>::Ptr &src,
                            typename ProbabilityGridmap<Tp, T>::Ptr &dst)
    {
        if(src != dst) {
            dst.reset(new ProbabilityGridmap<Tp, T>(*src));
        }
        std::for_each(dst->getData().begin(),
                      dst->getData().end(),
                      [](T &l){l = cslibs_math::common::LogOdds<T>::from(l);});

    }
};
}
}
}

#endif // CSLIBS_GRIDMAPS_CONVERT_PROBABILITY_GRIDMAP_HPP
