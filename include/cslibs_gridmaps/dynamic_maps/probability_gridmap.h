#ifndef CSLIBS_GRIDMAPS_DYNAMIC_PROBABILITY_GRIDMAP_H
#define CSLIBS_GRIDMAPS_DYNAMIC_PROBABILITY_GRIDMAP_H

#include <cslibs_gridmaps/dynamic_maps/gridmap.hpp>

namespace cslibs_gridmaps {
namespace dynamic_maps {
class EIGEN_ALIGN16 ProbabilityGridmap : public Gridmap<double>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<ProbabilityGridmap>;

    using Ptr = std::shared_ptr<ProbabilityGridmap>;

    ProbabilityGridmap(const pose_t &origin,
                       const double resolution,
                       const double chunk_resolution,
                       const double default_value = 0.5);

    ProbabilityGridmap(const ProbabilityGridmap &other);
    ProbabilityGridmap(ProbabilityGridmap &&other);

};
}
}

#endif // CSLIBS_GRIDMAPS_DYNAMIC_PROBABILITY_GRIDMAP_H
