#include <cslibs_gridmaps/static_maps/probability_gridmap.h>

namespace cslibs_gridmaps {
namespace static_maps {
ProbabilityGridmap::ProbabilityGridmap(const pose_t &origin,
                                       const double resolution,
                                       const std::size_t height,
                                       const std::size_t width,
                                       const double default_value) :
    Gridmap<double>(origin,
                    resolution,
                    height,
                    width,
                    default_value)
{
}

ProbabilityGridmap::ProbabilityGridmap(const ProbabilityGridmap &other) :
    Gridmap<double>(static_cast<const Gridmap<double>&>(other))
{
}

ProbabilityGridmap::ProbabilityGridmap(ProbabilityGridmap &&other) :
    Gridmap<double>(static_cast<Gridmap<double>&&>(other))
{
}
}
}
