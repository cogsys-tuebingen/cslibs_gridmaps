#include <cslibs_gridmaps/dynamic_maps/min_max_heightmap.h>

namespace cslibs_gridmaps {
namespace dynamic_maps {
MinMaxHeightmap::MinMaxHeightmap(const pose_t &origin,
                                       const double resolution,
                                       const double chunk_resolution,
                                       const double default_value) :
    Gridmap<double>(origin,
                    resolution,
                    chunk_resolution,
                    default_value),
    resolution_2_(resolution * resolution)
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
    line_iterator_t it = getLineIterator(sensor_xy, point_xy);
    const double dist_2d_2 = (point_xy - sensor_xy).length2();
    auto get_height = [this, &sensor_z, &point_z, &dist_2d_2](const double &traversed_2) {
        const double dist_curr_2 = traversed_2 * resolution_2_;
        const double ratio = std::sqrt(dist_curr_2 / dist_2d_2);
        return sensor_z + ratio * (point_z - sensor_z);
    };
    auto update_free = [](const double &map_height, const double &measured_height) {
        return std::isfinite(map_height) ? std::min(map_height, measured_height) : measured_height;
    };
    auto update_occupied = [](const double &map_height, const double &measured_height) {
        return std::isfinite(map_height) ? std::max(map_height, measured_height) : measured_height;
    };

    while (!it.done()) {
        const double height = get_height(static_cast<double>(it.traversed2()));
        *it = update_free(*it, height);
    }
    *it = update_occupied(*it, point_z);
}

}
}
