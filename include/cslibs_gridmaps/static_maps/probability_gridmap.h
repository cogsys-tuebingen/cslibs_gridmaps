#ifndef CSLIBS_GRIDMAPS_STATIC_PROBABILITY_GRIDMAP_H
#define CSLIBS_GRIDMAPS_STATIC_PROBABILITY_GRIDMAP_H

#include <cslibs_gridmaps/static_maps/gridmap.hpp>

namespace cslibs_gridmaps {
namespace static_maps {
template <typename Tp = double, typename T = double, typename AllocatorT = std::allocator<T>>
class /*EIGEN_ALIGN16*/ ProbabilityGridmap : public Gridmap<Tp, T, AllocatorT>
{
public:
//    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//    using allocator_t = Eigen::aligned_allocator<ProbabilityGridmap<Tp, T, AllocatorT>>;

    using Ptr = std::shared_ptr<ProbabilityGridmap<Tp, T, AllocatorT>>;
    using base_t = Gridmap<Tp, T, AllocatorT>;
    using pose_t = typename base_t::pose_t;

    explicit ProbabilityGridmap(const pose_t &origin,
                                const Tp resolution,
                                const std::size_t height,
                                const std::size_t width,
                                const T default_value = 0.5) :
        base_t(origin,
               resolution,
               height,
               width,
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

#endif // CSLIBS_GRIDMAPS_STATIC_PROBABILITY_GRIDMAP_H
