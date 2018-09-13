#include <cslibs_gridmaps/dynamic_maps/probability_gridmap.h>

namespace cslibs_gridmaps {
namespace dynamic_maps {
ProbabilityGridmap::ProbabilityGridmap(const pose_t &origin,
                                       const double resolution,
                                       const double chunk_resolution,
                                       const double default_value) :
    Gridmap<double>(origin,
                    resolution,
                    chunk_resolution,
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
