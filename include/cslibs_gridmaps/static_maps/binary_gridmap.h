#ifndef CSLIBS_GRIDMAPS_STATIC_BINARY_GRIDMAP_H
#define CSLIBS_GRIDMAPS_STATIC_BINARY_GRIDMAP_H

#include <cslibs_gridmaps/static_maps/gridmap.hpp>

namespace cslibs_gridmaps {
namespace static_maps {
class EIGEN_ALIGN16 BinaryGridmap : public Gridmap<int>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<BinaryGridmap>;

    using Ptr = std::shared_ptr<BinaryGridmap>;

    enum state_t {FREE = 0, OCCUPIED = 1};


    explicit BinaryGridmap(const pose_t &origin,
                           const double resolution,
                           const std::size_t height,
                           const std::size_t width,
                           const state_t default_value = FREE);

    BinaryGridmap(const BinaryGridmap &other);
    BinaryGridmap(BinaryGridmap &&other);

    double getRange(const cslibs_math_2d::Point2d &from,
                    cslibs_math_2d::Point2d &to) const;

    double getRange2(const cslibs_math_2d::Point2d &from,
                     cslibs_math_2d::Point2d &to) const;

    virtual bool validate(const cslibs_math_2d::Pose2d &p) const;

};
}
}

#endif /* CSLIBS_GRIDMAPS_STATIC_BINARY_GRIDMAP_H */
