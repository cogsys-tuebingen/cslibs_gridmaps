#ifndef CSLIBS_GRIDMAPS_STATIC_DISTANCE_GRIDMAP_H
#define CSLIBS_GRIDMAPS_STATIC_DISTANCE_GRIDMAP_H

#include <cslibs_gridmaps/static_maps/gridmap.hpp>

namespace cslibs_gridmaps {
namespace static_maps {
class DistanceGridmap : public Gridmap<double>
{
public:
    using Ptr = std::shared_ptr<DistanceGridmap>;

    explicit DistanceGridmap(const pose_t &origin,
                             const double resolution,
                             const double maximum_distance,
                             const std::size_t height,
                             const std::size_t width,
                             const double default_value = 2.0);

    DistanceGridmap(const DistanceGridmap &other) = default;

    using Gridmap<double>::at;
    double at(const cslibs_math_2d::Point2d &point) const override;
    double getMaximumDistance() const;


private:
    double maximum_distance_;

};
}
}

#endif /* CSLIBS_GRIDMAPS_STATIC_DISTANCE_GRIDMAP_H */
