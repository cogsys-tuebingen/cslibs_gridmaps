#ifndef CSLIBS_GRIDMAPS_DYNAMIC_DISTRIBUTION_HEIGHTMAP_H
#define CSLIBS_GRIDMAPS_DYNAMIC_DISTRIBUTION_HEIGHTMAP_H

#include <cslibs_gridmaps/dynamic_maps/gridmap.hpp>
#include <cslibs_math/statistics/distribution.hpp>

namespace cslibs_gridmaps {
namespace dynamic_maps {
template <typename Tp = double, typename T = double>
class EIGEN_ALIGN16 DistributionHeightmap : public Gridmap<Tp, cslibs_math::statistics::Distribution<1,T>>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<DistributionHeightmap<Tp, T>>;

    using Ptr = std::shared_ptr<DistributionHeightmap<Tp, T>>;
    using distribution_t = cslibs_math::statistics::Distribution<1,T>;
    using pose_t = typename Gridmap<Tp,cslibs_math::statistics::Distribution<1,T>>::pose_t;
    using point_t = typename Gridmap<Tp,cslibs_math::statistics::Distribution<1,T>>::point_t;

    DistributionHeightmap(const pose_t &origin,
                          const T resolution,
                          const T chunk_resolution,
                          const T max_height,
                          const distribution_t default_value = distribution_t()) :
        Gridmap<Tp, distribution_t>(origin,
                                    resolution,
                                    chunk_resolution,
                                    default_value),
        resolution_2_(resolution * resolution),
        max_height_(max_height)
    {
    }

    DistributionHeightmap(const DistributionHeightmap &other) :
        Gridmap<Tp, distribution_t>(static_cast<const Gridmap<Tp, distribution_t>&>(other)),
        resolution_2_(other.resolution_2_),
        max_height_(other.max_height_)
    {
    }
    DistributionHeightmap(DistributionHeightmap &&other) :
        Gridmap<Tp, distribution_t>(static_cast<Gridmap<Tp, distribution_t>&&>(other)),
        resolution_2_(other.resolution_2_),
        max_height_(other.max_height_)
    {
    }

    void insert(const point_t &sensor_xy, const T &sensor_z,
                const point_t &point_xy,  const T &point_z)
    {
        if (point_z > max_height_)
            return;

        const index_t index              = toIndex(point_xy);
        const index_t chunk_index        = toChunkIndex(index);
        const index_t local_chunk_index  = toLocalChunkIndex(index);

        typename chunk_t::handle_t chunk = getAllocateChunk(chunk_index);
        chunk->at(local_chunk_index) += point_z;
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

#endif // CSLIBS_GRIDMAPS_DYNAMIC_DISTRIBUTION_HEIGHTMAP_H
