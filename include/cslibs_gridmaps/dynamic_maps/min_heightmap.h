#ifndef CSLIBS_GRIDMAPS_DYNAMIC_MIN_HEIGHTMAP_H
#define CSLIBS_GRIDMAPS_DYNAMIC_MIN_HEIGHTMAP_H

#include <cslibs_gridmaps/dynamic_maps/gridmap.hpp>

namespace cslibs_gridmaps {
namespace dynamic_maps {
template <typename Tp = double, typename T = double, typename AllocatorT = std::allocator<T>>
class EIGEN_ALIGN16 MinHeightmap : public Gridmap<Tp, T, AllocatorT>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<MinHeightmap<Tp, T, AllocatorT>>;

    using Ptr = std::shared_ptr<MinHeightmap<Tp, T>>;
    using base_t = Gridmap<Tp, T, AllocatorT>;
    using pose_t = typename base_t::pose_t;
    using point_t = typename base_t::point_t;

    MinHeightmap(const pose_t &origin,
                 const Tp resolution,
                 const Tp chunk_resolution,
                 const T max_height,
                 const T default_value = std::numeric_limits<T>::infinity()) :
        base_t(origin,
               resolution,
               chunk_resolution,
               default_value),
        resolution_2_(resolution * resolution),
        max_height_(max_height)
    {
    }

    MinHeightmap(const MinHeightmap &other) :
        base_t(static_cast<const base_t&>(other)),
        resolution_2_(other.resolution_2_),
        max_height_(other.max_height_)
    {
    }
    MinHeightmap(MinHeightmap &&other) :
        base_t(static_cast<base_t&&>(other)),
        resolution_2_(other.resolution_2_),
        max_height_(other.max_height_)
    {
    }

    void insert(const point_t &sensor_xy, const Tp &sensor_z,
                const point_t &point_xy,  const Tp &point_z)
    {
        if (point_z > max_height_)
            return;

        auto update_occupied = [this](const T &map_height, const T &measured_height) {
            return std::isnormal(map_height) ? std::min(std::min(map_height, measured_height), max_height_) : std::min(measured_height, max_height_);
        };
        this->set(point_xy, update_occupied(this->get(point_xy), point_z));
    }

    T getMaxHeight() const
    {
        return max_height_;
    }

private:
    Tp resolution_2_;
    T max_height_;
};
}
}

#endif // CSLIBS_GRIDMAPS_DYNAMIC_MIN_HEIGHTMAP_H
