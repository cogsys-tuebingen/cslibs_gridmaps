#ifndef CSLIBS_GRIDMAPS_CONVERT_BINARY_GRIDMAP_HPP
#define CSLIBS_GRIDMAPS_CONVERT_BINARY_GRIDMAP_HPP

#include <cslibs_gridmaps/static_maps/binary_gridmap.h>

#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>

namespace cslibs_gridmaps {
namespace static_maps {
namespace conversion {
inline void from(const nav_msgs::OccupancyGrid &src,
                 BinaryGridmap::Ptr            &dst,
                 const double threshold = 1.0)
{
    assert(threshold <= 1.0);
    assert(threshold >= 0.0);

    cslibs_math_2d::Pose2d origin(src.info.origin.position.x,
                                  src.info.origin.position.y,
                                  tf::getYaw(src.info.origin.orientation));

    dst.reset(new BinaryGridmap(origin,
                                src.info.resolution,
                                src.info.height,
                                src.info.width));

    const int8_t  t = threshold * 100;
    std::transform(src.data.begin(), src.data.end(),
                  dst->getData().begin(),
                  [t](const int8_t p){int8_t occ = p == -1 ? 50 : p;
                                      return occ >= t ? BinaryGridmap::OCCUPIED : BinaryGridmap::FREE;});
}

inline void from(const nav_msgs::OccupancyGrid::ConstPtr &src,
                 BinaryGridmap::Ptr                      &dst,
                 const double threshold = 1.0)
{
    from(*src, dst, threshold);
}
}
}
}

#endif // CSLIBS_GRIDMAPS_CONVERT_BINARY_GRIDMAP_HPP
