#ifndef CSLIBS_GRIDMAPS_STATIC_PROBABILITY_GRIDMAP_H
#define CSLIBS_GRIDMAPS_STATIC_PROBABILITY_GRIDMAP_H

#include <cslibs_gridmaps/static_maps/gridmap.hpp>

namespace cslibs_gridmaps {
namespace static_maps {
class ProbabilityGridmap : public Gridmap<double>
{
public:
    using Ptr = std::shared_ptr<ProbabilityGridmap>;

    ProbabilityGridmap(const pose_t &origin,
                       const double resolution,
                       const std::size_t height,
                       const std::size_t width,
                       const std::string &frame_id,
                       const double default_value = 0.5);

    ProbabilityGridmap(const ProbabilityGridmap &other) = default;
};
}
}

#endif /* CSLIBS_GRIDMAPS_STATIC_PROBABILITY_GRIDMAP_H */
