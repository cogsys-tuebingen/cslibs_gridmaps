#include <cslibs_gridmaps/dynamic_maps/min_max_heightmap.h>

namespace cslibs_gridmaps {
namespace dynamic_maps {
MinMaxHeightmap::MinMaxHeightmap(const pose_t &origin,
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

MinMaxHeightmap::MinMaxHeightmap(const MinMaxHeightmap &other) :
    Gridmap<double>(static_cast<const Gridmap<double>&>(other))
{
}

MinMaxHeightmap::MinMaxHeightmap(MinMaxHeightmap &&other) :
    Gridmap<double>(static_cast<Gridmap<double>&&>(other))
{
}

void MinMaxHeightmap::insert(const point_t &sensor_xy, const double &sensor_z,
                             const point_t &point_xy, const double &point_z)
{
    auto update_occupied = [this](const double &map_height, const double &measured_height) {
        return std::isnormal(map_height) ? std::min(std::max(map_height, measured_height), max_height_) : std::min(measured_height, max_height_);
    };
    set(point_xy, update_occupied(get(point_xy), point_z));
}

double MinMaxHeightmap::getMaxHeight()
{
    return max_height_;
}

}
}
