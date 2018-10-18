#ifndef CSLIBS_GRIDMAPS_DYNAMIC_MIN_MAP_HEIGHTMAP_H
#define CSLIBS_GRIDMAPS_DYNAMIC_MIN_MAP_HEIGHTMAP_H

#include <cslibs_gridmaps/dynamic_maps/gridmap.hpp>

namespace cslibs_gridmaps {
namespace dynamic_maps {
class EIGEN_ALIGN16 MinMaxHeightmap : public Gridmap<double>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<MinMaxHeightmap>;

    using Ptr = std::shared_ptr<MinMaxHeightmap>;
    using point_t = cslibs_math_2d::Point2d;

    MinMaxHeightmap(const pose_t &origin,
                    const double resolution,
                    const double chunk_resolution,
                    const double default_value = std::numeric_limits<double>::infinity());

    MinMaxHeightmap(const MinMaxHeightmap &other);
    MinMaxHeightmap(MinMaxHeightmap &&other);

    inline void insert(const point_t &sensor_xy, const double &sensor_z,
                       const point_t &point_xy,  const double &point_z);

private:
    double resolution_2_;
};
}
}

#endif // CSLIBS_GRIDMAPS_DYNAMIC_MIN_MAP_HEIGHTMAP_H
