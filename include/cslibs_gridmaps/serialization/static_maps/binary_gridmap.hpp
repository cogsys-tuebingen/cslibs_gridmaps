#ifndef CSLIBS_GRIDMAPS_SERIALIZATION_STATIC_MAPS_BINARY_GRIDMAP_HPP
#define CSLIBS_GRIDMAPS_SERIALIZATION_STATIC_MAPS_BINARY_GRIDMAP_HPP

#include <cslibs_gridmaps/serialization/static_maps/gridmap.hpp>
#include <cslibs_gridmaps/static_maps/binary_gridmap.h>
#include <yaml-cpp/yaml.h>

namespace YAML {
template <>
struct convert<std::shared_ptr<cslibs_gridmaps::static_maps::BinaryGridmap>>
{
    static Node encode(const typename cslibs_gridmaps::static_maps::BinaryGridmap::Ptr &rhs)
    {
        return convert<cslibs_gridmaps::static_maps::Gridmap<int>::Ptr>::encode(rhs);
    }

    static bool decode(const Node& n, typename cslibs_gridmaps::static_maps::BinaryGridmap::Ptr &rhs)
    {
        if (!n.IsSequence() || n.size() != 5)
            return false;

        const std::size_t height = n[2].as<std::size_t>();
        const std::size_t width  = n[3].as<std::size_t>();
        rhs.reset(new cslibs_gridmaps::static_maps::BinaryGridmap(
                    n[0].as<cslibs_math_2d::Pose2d>(), n[1].as<double>(), height, width));

        std::vector<int> data = n[4].as<std::vector<int>>();
        if (data.size() != height * width)
            return false;

        for (std::size_t p = 0 ; p < (width * height); ++ p)
            rhs->at(p) = data[p];

        return true;
    }
};
}

#endif // CSLIBS_GRIDMAPS_SERIALIZATION_STATIC_MAPS_BINARY_GRIDMAP_HPP
