#include <cslibs_gridmaps/static_maps/probability_gridmap.h>

namespace cslibs_gridmaps {
namespace static_maps {
ProbabilityGridmap::ProbabilityGridmap(const pose_t &origin,
                                       const double resolution,
                                       const std::size_t height,
                                       const std::size_t width,
                                       const std::string &frame_id,
                                       const double default_value) :
    Gridmap<double>(origin,
                    resolution,
                    height,
                    width,
                    default_value,
                    frame_id)
{
}
}
}
