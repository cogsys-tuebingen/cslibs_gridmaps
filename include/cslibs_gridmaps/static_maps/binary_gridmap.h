#ifndef CSLIBS_GRIDMAPS_STATIC_BINARY_GRIDMAP_H
#define CSLIBS_GRIDMAPS_STATIC_BINARY_GRIDMAP_H

#include <cslibs_gridmaps/static_maps/gridmap.hpp>

namespace cslibs_gridmaps {
namespace static_maps {
template <typename Tp = double>
class EIGEN_ALIGN16 BinaryGridmap : public Gridmap<Tp, int>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<BinaryGridmap<Tp>>;

    using Ptr = std::shared_ptr<BinaryGridmap<Tp>>;
    using pose_t = typename Gridmap<Tp, int>::pose_t;
    using point_t = typename Gridmap<Tp, int>::point_t;
    using index_t = typename Gridmap<Tp, int>::index_t;
    using const_line_iterator_t = typename Gridmap<Tp, int>::const_line_iterator_t;

    enum state_t {FREE = 0, OCCUPIED = 1};


    explicit BinaryGridmap(const pose_t &origin,
                           const Tp resolution,
                           const std::size_t height,
                           const std::size_t width,
                           const state_t default_value = FREE) :
        Gridmap<Tp,int>(origin,
                        resolution,
                        height,
                        width,
                        default_value)
    {
    }

    BinaryGridmap(const BinaryGridmap &other) :
        Gridmap<Tp,int>(static_cast<const Gridmap<Tp,int>&>(other))
    {
    }
    BinaryGridmap(BinaryGridmap &&other) :
        Gridmap<Tp,int>(static_cast<Gridmap<Tp,int>&&>(other))
    {
    }

    Tp getRange(const point_t &from,
                point_t &to) const
    {
        const_line_iterator_t it = getConstLineIterator(from, to);
        while (it.iterate()) {
            if (*it) {
                fromIndex({{it.x(), it.y()}}, to);
                return distance(from, to);
            }
        }

        if (it.invalid() || *it) {
            fromIndex({{it.x(), it.y()}}, to);
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
        if (toIndex(p.translation(), index))
            return this->at(static_cast<std::size_t>(index[0]), static_cast<std::size_t>(index[1])) == 0;
        return false;
    }
};
}
}

#endif // CSLIBS_GRIDMAPS_STATIC_BINARY_GRIDMAP_H
