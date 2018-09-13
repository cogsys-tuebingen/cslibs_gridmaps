#ifndef CSLIBS_GRIDMAPS_STATIC_LIKELIHOOD_FIELD_GRIDMAP_H
#define CSLIBS_GRIDMAPS_STATIC_LIKELIHOOD_FIELD_GRIDMAP_H

#include <cslibs_gridmaps/static_maps/gridmap.hpp>

namespace cslibs_gridmaps {
namespace static_maps {
class EIGEN_ALIGN16 LikelihoodFieldGridmap : public Gridmap<double>
{
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using allocator_t = Eigen::aligned_allocator<LikelihoodFieldGridmap>;
    using Ptr = std::shared_ptr<LikelihoodFieldGridmap>;

    explicit LikelihoodFieldGridmap(const pose_t &origin,
                                    const double resolution,
                                    const std::size_t height,
                                    const std::size_t width,
                                    const double maximum_distance,
                                    const double sigma_hit,
                                    const double default_value = 0.0);

    LikelihoodFieldGridmap(const LikelihoodFieldGridmap &other);
    LikelihoodFieldGridmap(LikelihoodFieldGridmap &&other);


    using Gridmap<double>::at;
    double at(const cslibs_math_2d::Point2d &point) const override;

    double getSigmaHit() const;
    double getMaximumDistance() const;

private:
    const double sigma_hit_;
    const double maximum_distance_;
};
}
}

#endif // CSLIBS_GRIDMAPS_STATIC_LIKELIHOOD_FIELD_GRIDMAP_H
