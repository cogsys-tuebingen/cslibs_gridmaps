#ifndef CSLIBS_GRIDMAPS_SERIALIZATION_DYNAMIC_MAPS_DISTRIBUTION_HEIGHTMAP_HPP
#define CSLIBS_GRIDMAPS_SERIALIZATION_DYNAMIC_MAPS_DISTRIBUTION_HEIGHTMAP_HPP

#include <cslibs_gridmaps/serialization/dynamic_maps/gridmap.hpp>
#include <cslibs_gridmaps/dynamic_maps/distribution_heightmap.h>
#include <yaml-cpp/yaml.h>

namespace YAML {
template <>
struct convert<std::shared_ptr<cslibs_gridmaps::dynamic_maps::DistributionHeightmap>>
{
    using distribution_t = cslibs_math::statistics::Distribution<1>;
    static Node encode(const typename cslibs_gridmaps::dynamic_maps::DistributionHeightmap::Ptr &rhs)
    {
        return convert<cslibs_gridmaps::dynamic_maps::Gridmap<distribution_t>::Ptr>::encode(rhs);
    }

    static bool decode(const Node& n, typename cslibs_gridmaps::dynamic_maps::DistributionHeightmap::Ptr &rhs)
    {
        if (!n.IsSequence() || n.size() < 4)
            return false;

        const double resolution       = n[1].as<double>();
        const double chunk_resolution = resolution * (static_cast<double>(n[2].as<std::size_t>()) + 0.5);
        rhs.reset(new cslibs_gridmaps::dynamic_maps::DistributionHeightmap(
                    n[0].as<cslibs_math_2d::Pose2d>(), resolution, chunk_resolution, n[3].as<double>()));

        for (std::size_t p = 4 ; p < n.size() ; ++ p) {
            cslibs_gridmaps::dynamic_maps::IndexedChunk<distribution_t, 2> ci =
                    n[p].as<cslibs_gridmaps::dynamic_maps::IndexedChunk<distribution_t, 2>>();
            rhs->getAllocateChunk(ci.index_).data() = ci.chunk_;
        }

        return true;
    }
};
}

#endif // CSLIBS_GRIDMAPS_SERIALIZATION_DYNAMIC_MAPS_DISTRIBUTION_HEIGHTMAP_HPP
