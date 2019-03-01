#ifndef CSLIBS_GRIDMAPS_DYNAMIC_PROBABILITY_GRIDMAP_H
#define CSLIBS_GRIDMAPS_DYNAMIC_PROBABILITY_GRIDMAP_H

#include <cslibs_gridmaps/dynamic_maps/gridmap.hpp>

namespace cslibs_gridmaps {
namespace dynamic_maps {
template <typename Tp = double, typename T = double>
class EIGEN_ALIGN16 ProbabilityGridmap : public Gridmap<Tp, T>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<ProbabilityGridmap<Tp, T>>;

    using Ptr = std::shared_ptr<ProbabilityGridmap<Tp, T>>;
    using pose_t = typename Gridmap<Tp, T>::pose_t;
    using point_t = typename Gridmap<Tp, T>::point_t;

    ProbabilityGridmap(const pose_t &origin,
                       const Tp resolution,
                       const Tp chunk_resolution,
                       const T default_value = 0.5) :
        Gridmap<Tp, T>(origin,
                       resolution,
                       chunk_resolution,
                       default_value)
    {
    }

    ProbabilityGridmap(const ProbabilityGridmap &other) :
        Gridmap<Tp, T>(static_cast<const Gridmap<Tp, T>&>(other))
    {
    }
    ProbabilityGridmap(ProbabilityGridmap &&other) :
        Gridmap<Tp, T>(static_cast<Gridmap<Tp, T>&&>(other))
    {
    }
};
}
}

#endif // CSLIBS_GRIDMAPS_DYNAMIC_PROBABILITY_GRIDMAP_H
