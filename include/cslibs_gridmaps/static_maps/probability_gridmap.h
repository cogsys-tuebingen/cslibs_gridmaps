#ifndef CSLIBS_GRIDMAPS_STATIC_PROBABILITY_GRIDMAP_H
#define CSLIBS_GRIDMAPS_STATIC_PROBABILITY_GRIDMAP_H

#include <cslibs_gridmaps/static_maps/gridmap.hpp>

namespace cslibs_gridmaps {
namespace static_maps {
class EIGEN_ALIGN16 ProbabilityGridmap : public Gridmap<double>
{
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using allocator_t = Eigen::aligned_allocator<ProbabilityGridmap>;

    using Ptr = std::shared_ptr<ProbabilityGridmap>;

    explicit ProbabilityGridmap(const pose_t &origin,
                                const double resolution,
                                const std::size_t height,
                                const std::size_t width,
                                const double default_value = 0.5);

    ProbabilityGridmap(const ProbabilityGridmap &other) = default;
};
}
}

#endif /* CSLIBS_GRIDMAPS_STATIC_PROBABILITY_GRIDMAP_H */
