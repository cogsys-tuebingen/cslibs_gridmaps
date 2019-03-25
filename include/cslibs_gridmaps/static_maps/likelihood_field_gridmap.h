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
    using base_t = Gridmap<Tp, T, AllocatorT>;
    using pose_t = typename base_t::pose_t;
    using point_t = typename base_t::point_t;
    using index_t = typename base_t::index_t;

    explicit LikelihoodFieldGridmap(const pose_t &origin,
                                    const Tp resolution,
                                    const std::size_t height,
                                    const std::size_t width,
                                    const T maximum_distance,
                                    const T sigma_hit,
                                    const T default_value = 0.0) :
        base_t(origin,
               resolution,
               height,
               width,
               default_value),
        sigma_hit_(sigma_hit),
        maximum_distance_(maximum_distance)
    {
    }

    LikelihoodFieldGridmap(const LikelihoodFieldGridmap &other) :
        base_t(static_cast<const base_t&>(other)),
        sigma_hit_(other.sigma_hit_),
        maximum_distance_(other.maximum_distance_)
    {
    }
    LikelihoodFieldGridmap(LikelihoodFieldGridmap &&other) :
        base_t(static_cast<base_t&&>(other)),
        sigma_hit_(other.sigma_hit_),
        maximum_distance_(other.maximum_distance_)
    {
    }

    using base_t::at;
    T at(const point_t &point) const override
    {
        index_t i;
        this->toIndex(point, i);
        if(this->invalid(i))
            return 0.0;
        return base_t::at(i[0], i[1]);
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
