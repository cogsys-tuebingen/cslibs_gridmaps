#include <cslibs_gridmaps/static_maps/likelihood_field_gridmap.h>
#include <cslibs_gridmaps/static_maps/algorithms/distance_transform.hpp>

namespace cslibs_gridmaps {
namespace static_maps {
LikelihoodFieldGridmap::LikelihoodFieldGridmap(const pose_t &origin,
                                               const double resolution,
                                               const std::size_t height,
                                               const std::size_t width,
                                               const double maximum_distance,
                                               const double sigma_hit,
                                               const double default_value) :
    Gridmap<double>(origin,
                    resolution,
                    height,
                    width,
                    default_value),
    sigma_hit_(sigma_hit),
    maximum_distance_(maximum_distance)
{
}

double LikelihoodFieldGridmap::at(const cslibs_math_2d::Point2d &point) const
{
    index_t i;
    toIndex(point, i);
    if(invalid(i))
        return 0.0;
    return Gridmap<double>::at(i[0], i[1]);
}

double LikelihoodFieldGridmap::getSigmaHit() const
{
    return sigma_hit_;
}
}
}
