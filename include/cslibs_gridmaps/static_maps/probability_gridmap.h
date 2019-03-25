#ifndef CSLIBS_GRIDMAPS_STATIC_PROBABILITY_GRIDMAP_H
#define CSLIBS_GRIDMAPS_STATIC_PROBABILITY_GRIDMAP_H

#include <cslibs_gridmaps/static_maps/gridmap.hpp>

namespace cslibs_gridmaps {
namespace static_maps {
template <typename Tp = double, typename T = double, typename AllocatorT = std::allocator<T>>
class EIGEN_ALIGN16 ProbabilityGridmap : public Gridmap<Tp, T, AllocatorT>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<ProbabilityGridmap<Tp, T, AllocatorT>>;

    using Ptr = std::shared_ptr<ProbabilityGridmap<Tp, T, AllocatorT>>;
    using pose_t = typename Gridmap<Tp, T, AllocatorT>::pose_t;

    explicit ProbabilityGridmap(const pose_t &origin,
                                const Tp resolution,
                                const std::size_t height,
                                const std::size_t width,
                                const T default_value = 0.5) :
        Gridmap<Tp,T,AllocatorT>(origin,
                                 resolution,
                                 height,
                                 width,
                                 default_value)
    {
    }

    ProbabilityGridmap(const ProbabilityGridmap &other) :
        Gridmap<Tp,T,AllocatorT>(static_cast<const Gridmap<Tp,T,AllocatorT>&>(other))
    {
    }

    ProbabilityGridmap(ProbabilityGridmap &&other) :
        Gridmap<Tp,T,AllocatorT>(static_cast<Gridmap<Tp,T,AllocatorT>&&>(other))
    {
    }
};
}
}

#endif // CSLIBS_GRIDMAPS_STATIC_PROBABILITY_GRIDMAP_H
