#include <cslibs_gridmaps/static_maps/binary_gridmap.h>

namespace cslibs_gridmaps {
namespace static_maps {
BinaryGridmap::BinaryGridmap(const pose_t &origin,
                             const double resolution,
                             const std::size_t height,
                             const std::size_t width,
                             const state_t default_value) :
    Gridmap<int>(origin,
                 resolution,
                 height,
                 width,
                 default_value)
{
}

double BinaryGridmap::getRange(const cslibs_math_2d::Point2d &from,
                               cslibs_math_2d::Point2d &to) const
{
    const_line_iterator_t it = getConstLineIterator(from, to);
    while (it.iterate()) {
        if (*it)
            return std::sqrt(static_cast<double>(it.traversed2())) * resolution_;
    }

    if (*it || it.invalid())
        return std::sqrt(static_cast<double>(it.traversed2())) * resolution_;
    return std::numeric_limits<double>::max();
}

double BinaryGridmap::getRange2(const cslibs_math_2d::Point2d &from,
                                cslibs_math_2d::Point2d &to) const
{
    const_line_iterator_t it = getConstLineIterator(from, to);
    while (it.iterate()) {
        if (*it)
            return static_cast<double>(it.traversed2()) * resolution_ * resolution_;
    }
    if (it.invalid())
        return static_cast<double>(it.traversed2()) * resolution_ * resolution_;
    return std::numeric_limits<double>::max();
}

bool BinaryGridmap::validate(const cslibs_math_2d::Pose2d &p) const
{
    index_t index;
    if(toIndex(p.translation(), index))
        return at(static_cast<std::size_t>(index[0]), static_cast<std::size_t>(index[1])) == 0;
    return false;
}
}
}
