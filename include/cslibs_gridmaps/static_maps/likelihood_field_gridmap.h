#ifndef CSLIBS_GRIDMAPS_STATIC_LIKELIHOOD_FIELD_GRIDMAP_H
#define CSLIBS_GRIDMAPS_STATIC_LIKELIHOOD_FIELD_GRIDMAP_H

#include <cslibs_gridmaps/static_maps/gridmap.hpp>

namespace cslibs_gridmaps {
namespace static_maps {
template <typename Tp = double, typename T = double, typename AllocatorT = std::allocator<T>>
class EIGEN_ALIGN16 LikelihoodFieldGridmap : public Gridmap<Tp, T, AllocatorT>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<LikelihoodFieldGridmap<Tp, T, AllocatorT>>;

    using Ptr = std::shared_ptr<LikelihoodFieldGridmap<Tp, T>>;
    using pose_t = typename Gridmap<Tp, T, AllocatorT>::pose_t;
    using point_t = typename Gridmap<Tp, T, AllocatorT>::point_t;
    using index_t = typename Gridmap<Tp, T, AllocatorT>::index_t;

    explicit LikelihoodFieldGridmap(const pose_t &origin,
                                    const Tp resolution,
                                    const std::size_t height,
                                    const std::size_t width,
                                    const T maximum_distance,
                                    const T sigma_hit,
                                    const T default_value = 0.0) :
        Gridmap<Tp,T, AllocatorT>(origin,
                                  resolution,
                                  height,
                                  width,
                                  default_value),
        sigma_hit_(sigma_hit),
        maximum_distance_(maximum_distance)
    {
    }

    LikelihoodFieldGridmap(const LikelihoodFieldGridmap &other) :
        Gridmap<Tp,T,AllocatorT>(static_cast<const Gridmap<Tp,T,AllocatorT>&>(other)),
        sigma_hit_(other.sigma_hit_),
        maximum_distance_(other.maximum_distance_)
    {
    }
    LikelihoodFieldGridmap(LikelihoodFieldGridmap &&other) :
        Gridmap<Tp,T,AllocatorT>(static_cast<Gridmap<Tp,T,AllocatorT>&&>(other)),
        sigma_hit_(other.sigma_hit_),
        maximum_distance_(other.maximum_distance_)
    {
    }

    using Gridmap<Tp, T, AllocatorT>::at;
    T at(const point_t &point) const override
    {
        index_t i;
        this->toIndex(point, i);
        if(this->invalid(i))
            return 0.0;
        return Gridmap<Tp,T,AllocatorT>::at(i[0], i[1]);
    }

    T getSigmaHit() const
    {
        return sigma_hit_;
    }
    T getMaximumDistance() const
    {
        return maximum_distance_;
    }

private:
    const T sigma_hit_;
    const T maximum_distance_;
};
}
}

#endif // CSLIBS_GRIDMAPS_STATIC_LIKELIHOOD_FIELD_GRIDMAP_H
