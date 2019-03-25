#ifndef CSLIBS_GRIDMAPS_DYNAMIC_PROBABILITY_GRIDMAP_H
#define CSLIBS_GRIDMAPS_DYNAMIC_PROBABILITY_GRIDMAP_H

#include <cslibs_gridmaps/dynamic_maps/gridmap.hpp>

namespace cslibs_gridmaps {
namespace dynamic_maps {
template <typename Tp = double, typename T = double, typename AllocatorT = std::allocator<T>>
class EIGEN_ALIGN16 ProbabilityGridmap : public Gridmap<Tp, T, AllocatorT>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<ProbabilityGridmap<Tp, T, AllocatorT>>;

    using Ptr = std::shared_ptr<ProbabilityGridmap<Tp, T, AllocatorT>>;
    using base_t = Gridmap<Tp, T, AllocatorT>;
    using pose_t = typename base_t::pose_t;
    using point_t = typename base_t::point_t;

    ProbabilityGridmap(const pose_t &origin,
                       const Tp resolution,
                       const Tp chunk_resolution,
                       const T default_value = 0.5) :
        base_t(origin,
               resolution,
               chunk_resolution,
               default_value)
    {
    }

    ProbabilityGridmap(const ProbabilityGridmap &other) :
        base_t(static_cast<const base_t&>(other))
    {
    }
    ProbabilityGridmap(ProbabilityGridmap &&other) :
        base_t(static_cast<base_t&&>(other))
    {
    }
};
}
}

#endif // CSLIBS_GRIDMAPS_DYNAMIC_PROBABILITY_GRIDMAP_H
