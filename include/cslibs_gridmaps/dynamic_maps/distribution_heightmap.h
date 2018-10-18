#ifndef CSLIBS_GRIDMAPS_DYNAMIC_DISTRIBUTION_HEIGHTMAP_H
#define CSLIBS_GRIDMAPS_DYNAMIC_DISTRIBUTION_HEIGHTMAP_H

#include <cslibs_gridmaps/dynamic_maps/gridmap.hpp>
#include <cslibs_math/statistics/distribution.hpp>

namespace cslibs_gridmaps {
namespace dynamic_maps {
class EIGEN_ALIGN16 DistributionHeightmap : public Gridmap<cslibs_math::statistics::Distribution<1>>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<DistributionHeightmap>;

    using Ptr = std::shared_ptr<DistributionHeightmap>;
    using point_t = cslibs_math_2d::Point2d;
    using distribution_t = cslibs_math::statistics::Distribution<1>;

    DistributionHeightmap(const pose_t &origin,
                          const double resolution,
                          const double chunk_resolution,
                          const distribution_t default_value = distribution_t());

    DistributionHeightmap(const DistributionHeightmap &other);
    DistributionHeightmap(DistributionHeightmap &&other);

    void insert(const point_t &sensor_xy, const double &sensor_z,
                const point_t &point_xy,  const double &point_z);

private:
    double resolution_2_;
};
}
}

#endif // CSLIBS_GRIDMAPS_DYNAMIC_DISTRIBUTION_HEIGHTMAP_H
