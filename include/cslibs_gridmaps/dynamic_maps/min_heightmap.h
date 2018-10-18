#ifndef CSLIBS_GRIDMAPS_DYNAMIC_MIN_HEIGHTMAP_H
#define CSLIBS_GRIDMAPS_DYNAMIC_MIN_HEIGHTMAP_H

#include <cslibs_gridmaps/dynamic_maps/gridmap.hpp>

namespace cslibs_gridmaps {
namespace dynamic_maps {
class EIGEN_ALIGN16 MinHeightmap : public Gridmap<double>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<MinHeightmap>;

    using Ptr = std::shared_ptr<MinHeightmap>;
    using point_t = cslibs_math_2d::Point2d;

    MinHeightmap(const pose_t &origin,
                    const double resolution,
                    const double chunk_resolution,
                    const double max_height,
                    const double default_value = std::numeric_limits<double>::infinity());

    MinHeightmap(const MinHeightmap &other);
    MinHeightmap(MinHeightmap &&other);

    void insert(const point_t &sensor_xy, const double &sensor_z,
                const point_t &point_xy,  const double &point_z);

    double getMaxHeight() const;

private:
    double resolution_2_;
    double max_height_;
};
}
}

#endif // CSLIBS_GRIDMAPS_DYNAMIC_MIN_HEIGHTMAP_H
