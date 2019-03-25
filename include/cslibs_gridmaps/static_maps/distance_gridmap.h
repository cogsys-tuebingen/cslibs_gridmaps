#ifndef CSLIBS_GRIDMAPS_STATIC_DISTANCE_GRIDMAP_H
#define CSLIBS_GRIDMAPS_STATIC_DISTANCE_GRIDMAP_H

#include <cslibs_gridmaps/static_maps/gridmap.hpp>

namespace cslibs_gridmaps {
namespace static_maps {
template <typename Tp = double, typename T = double, typename AllocatorT = std::allocator<T>>
class EIGEN_ALIGN16 DistanceGridmap : public Gridmap<Tp, T, AllocatorT>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<DistanceGridmap<Tp, T, AllocatorT>>;

    using Ptr = std::shared_ptr<DistanceGridmap<Tp, T, AllocatorT>>;
    using base_t = Gridmap<Tp, T, AllocatorT>;
    using pose_t = typename base_t::pose_t;
    using point_t = typename base_t::point_t;
    using index_t = typename base_t::index_t;

    explicit DistanceGridmap(const pose_t &origin,
                             const Tp resolution,
                             const T maximum_distance,
                             const std::size_t height,
                             const std::size_t width,
                             const T default_value = 2.0) :
        base_t(origin,
               resolution,
               height,
               width,
               default_value),
        maximum_distance_(maximum_distance)
    {
    }

    DistanceGridmap(const DistanceGridmap &other) :
        base_t(static_cast<const base_t&>(other)),
        maximum_distance_(other.maximum_distance_)
    {
    }

    DistanceGridmap(DistanceGridmap &&other) :
        base_t(static_cast<base_t&&>(other)),
        maximum_distance_(other.maximum_distance_)
    {
    }

    using base_t::at;
    T at(const point_t &point) const override
    {
        index_t i;
        this->toIndex(point, i);
        if(this->invalid(i))
            return maximum_distance_;
        return base_t::at(i[0], i[1]);
    }

    T getMaximumDistance() const
    {
        return maximum_distance_;
    }

private:
    T maximum_distance_;
};
}
}

#endif // CSLIBS_GRIDMAPS_STATIC_DISTANCE_GRIDMAP_H
