#ifndef CSLIBS_GRIDMAPS_STATIC_BINARY_GRIDMAP_H
#define CSLIBS_GRIDMAPS_STATIC_BINARY_GRIDMAP_H

#include <cslibs_gridmaps/static_maps/gridmap.hpp>

namespace cslibs_gridmaps {
namespace static_maps {
template <typename Tp = double>
class EIGEN_ALIGN16 BinaryGridmap : public Gridmap<Tp, int, std::allocator<int>>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<BinaryGridmap<Tp>>;

    using Ptr = std::shared_ptr<BinaryGridmap<Tp>>;
    using base_t = Gridmap<Tp, int, std::allocator<int>>;
    using pose_t = typename base_t::pose_t;
    using point_t = typename base_t::point_t;
    using index_t = typename base_t::index_t;
    using const_line_iterator_t = typename base_t::const_line_iterator_t;

    enum state_t {FREE = 0, OCCUPIED = 1};


    explicit BinaryGridmap(const pose_t &origin,
                           const Tp resolution,
                           const std::size_t height,
                           const std::size_t width,
                           const state_t default_value = FREE) :
        base_t(origin,
               resolution,
               height,
               width,
               default_value)
    {
    }

    BinaryGridmap(const BinaryGridmap &other) :
        base_t(static_cast<const base_t&>(other))
    {
    }
    BinaryGridmap(BinaryGridmap &&other) :
        base_t(static_cast<base_t&&>(other))
    {
    }

    Tp getRange(const point_t &from,
                point_t &to) const
    {
        const_line_iterator_t it = this->getConstLineIterator(from, to);
        while (it.iterate()) {
            if (*it) {
                this->fromIndex({{it.x(), it.y()}}, to);
                return distance(from, to);
            }
        }

        if (it.invalid() || *it) {
            this->fromIndex({{it.x(), it.y()}}, to);
            return distance(from, to);
        }
        return std::numeric_limits<Tp>::max();
    }

    Tp getRange2(const point_t &from,
                 point_t &to) const
    {
        const_line_iterator_t it = getConstLineIterator(from, to);
        while (it.iterate()) {
            if (*it) {
                fromIndex({{it.x(), it.y()}}, to);
                return distance2(from, to);
            }
        }

        if (it.invalid() || *it) {
            fromIndex({{it.x(), it.y()}}, to);
            return distance2(from, to);
        }
        return std::numeric_limits<Tp>::max();
    }

    virtual bool validate(const pose_t &p) const
    {
        index_t index;
        if (this->toIndex(p.translation(), index))
            return this->at(static_cast<std::size_t>(index[0]), static_cast<std::size_t>(index[1])) == 0;
        return false;
    }
};
}
}

#endif // CSLIBS_GRIDMAPS_STATIC_BINARY_GRIDMAP_H
