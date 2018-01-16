#ifndef CSLIBS_GRIDMAPS_CONVERT_LIKELIHOOD_FIELD_GRIDMAP_HPP
#define CSLIBS_GRIDMAPS_CONVERT_LIKELIHOOD_FIELD_GRIDMAP_HPP

#include <cslibs_gridmaps/static_maps/likelihood_field_gridmap.h>
#include <cslibs_gridmaps/static_maps/algorithms/distance_transform.hpp>

#include <tf/tf.h>
#include <nav_msgs/OccupancyGrid.h>

namespace cslibs_gridmaps {
namespace static_maps {
namespace conversion {
inline void from(const nav_msgs::OccupancyGrid &src,
                 LikelihoodFieldGridmap::Ptr &dst,
                 const double maximum_distance = 2.0,
                 const double sigma_hit        = 0.5,
                 const double threshold        = 1.0)
{
    assert(threshold <= 1.0);
    assert(threshold >= 0.0);
    const double exp_factor_hit = (0.5 * 1.0 / (sigma_hit * sigma_hit));

    cslibs_math_2d::Pose2d origin(src.info.origin.position.x,
                                     src.info.origin.position.y,
                                     tf::getYaw(src.info.origin.orientation));

    dst.reset(new LikelihoodFieldGridmap(origin,
                                         src.info.resolution,
                                         src.info.height,
                                         src.info.width,
                                         maximum_distance,
                                         sigma_hit));

    std::vector<int8_t> occ(src.data.size());
    std::transform(src.data.begin(),
                   src.data.end(),
                   occ.begin(),
                   [](const int8_t p){return p != -1 ? p : 50;});

    /// 1.) calculate the distances
    algorithms::DistanceTransform<int8_t> distance_transform(src.info.resolution,
                                                             maximum_distance,
                                                             static_cast<int8_t>(threshold * 100));
    distance_transform.apply(occ,
                             dst->getWidth(),
                             dst->getData());

    /// 2.) pre-calculation of the hit likelihoods
    std::for_each(dst->getData().begin(),
                  dst->getData().end(),
                  [exp_factor_hit] (double &z) {z = std::exp(-z * z * exp_factor_hit);});
}

inline void from(const nav_msgs::OccupancyGrid::Ptr &src,
                 LikelihoodFieldGridmap::Ptr   &dst,
                 const double maximum_distance = 2.0,
                 const double sigma_hit        = 0.5,
                 const double threshold        = 1.0)
{
    from(*src, dst, maximum_distance, sigma_hit, threshold);
}

inline void from(const LikelihoodFieldGridmap::Ptr &src,
                 nav_msgs::OccupancyGrid::Ptr &dst)
{
    if (!src)
        return;

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
                   [](const double p){return 100.0 * p;});
}
}
}
}

#endif // CSLIBS_GRIDMAPS_CONVERT_LIKELIHOOD_FIELD_GRIDMAP_HPP
