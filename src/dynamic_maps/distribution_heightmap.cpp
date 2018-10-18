#include <cslibs_gridmaps/dynamic_maps/distribution_heightmap.h>

namespace cslibs_gridmaps {
namespace dynamic_maps {
DistributionHeightmap::DistributionHeightmap(const pose_t &origin,
                                             const double resolution,
                                             const double chunk_resolution,
                                             const distribution_t default_value) :
    Gridmap<distribution_t>(origin,
                            resolution,
                            chunk_resolution,
                            default_value),
    resolution_2_(resolution * resolution)
{
}

DistributionHeightmap::DistributionHeightmap(const DistributionHeightmap &other) :
    Gridmap<distribution_t>(static_cast<const Gridmap<distribution_t>&>(other))
{
}

DistributionHeightmap::DistributionHeightmap(DistributionHeightmap &&other) :
    Gridmap<distribution_t>(static_cast<Gridmap<distribution_t>&&>(other))
{
}

void DistributionHeightmap::insert(const point_t &sensor_xy, const double &sensor_z,
                                   const point_t &point_xy, const double &point_z)
{
    const index_t index              = toIndex(point_xy);
    const index_t chunk_index        = toChunkIndex(index);
    const index_t local_chunk_index  = toLocalChunkIndex(index);

    typename chunk_t::handle_t chunk = getAllocateChunk(chunk_index);
    chunk->at(local_chunk_index) += point_z;
}

}
}
