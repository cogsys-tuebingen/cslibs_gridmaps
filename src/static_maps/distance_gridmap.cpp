#include <cslibs_gridmaps/static_maps/distance_gridmap.h>
#include <cslibs_gridmaps/static_maps/algorithms/distance_transform.hpp>

namespace cslibs_gridmaps {
namespace static_maps {
DistanceGridmap::DistanceGridmap(const pose_t &origin,
                                 const double resolution,
                                 const double maximum_distance,
                                 const std::size_t height,
                                 const std::size_t width,
                                 const double default_value) :
    Gridmap<double>(origin,
                    resolution,
                    height,
                    width,
                    default_value),
    maximum_distance_(maximum_distance)
{
}

double DistanceGridmap::at(const cslibs_math_2d::Point2d &point) const
{
    index_t i;
    toIndex(point, i);
    if(invalid(i))
        return maximum_distance_;
    return Gridmap<double>::at(i[0], i[1]);
}

double DistanceGridmap::getMaximumDistance() const
{
    return maximum_distance_;
}
}
}
