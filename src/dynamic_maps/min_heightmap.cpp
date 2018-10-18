#include <cslibs_gridmaps/dynamic_maps/min_heightmap.h>

namespace cslibs_gridmaps {
namespace dynamic_maps {
MinHeightmap::MinHeightmap(const pose_t &origin,
                                 const double resolution,
                                 const double chunk_resolution,
                                 const double max_height,
                                 const double default_value) :
    Gridmap<double>(origin,
                    resolution,
                    chunk_resolution,
                    default_value),
    resolution_2_(resolution * resolution),
    max_height_(max_height)
{
}

MinHeightmap::MinHeightmap(const MinHeightmap &other) :
    Gridmap<double>(static_cast<const Gridmap<double>&>(other))
{
}

MinHeightmap::MinHeightmap(MinHeightmap &&other) :
    Gridmap<double>(static_cast<Gridmap<double>&&>(other))
{
}

void MinHeightmap::insert(const point_t &sensor_xy, const double &sensor_z,
                             const point_t &point_xy, const double &point_z)
{
    if (point_z > max_height_)
        return;

    auto update_occupied = [this](const double &map_height, const double &measured_height) {
        return std::isnormal(map_height) ? std::min(std::min(map_height, measured_height), max_height_) : std::min(measured_height, max_height_);
    };
    set(point_xy, update_occupied(get(point_xy), point_z));
}

double MinHeightmap::getMaxHeight() const
{
    return max_height_;
}

}
}
