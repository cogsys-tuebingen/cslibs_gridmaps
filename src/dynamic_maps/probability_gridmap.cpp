#include <cslibs_gridmaps/dynamic_maps/probability_gridmap.h>

namespace cslibs_gridmaps {
namespace dynamic_maps {
ProbabilityGridmap::ProbabilityGridmap(const pose_t &origin,
                                       const double resolution,
                                       const double chunk_resolution,
                                       const std::string &frame_id,
                                       const double default_value) :
    Gridmap<double>(origin,
                    resolution,
                    chunk_resolution,
                    frame_id,
                    default_value)
{
}
}
}
