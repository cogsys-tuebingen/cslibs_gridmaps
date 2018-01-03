#ifndef CSLIBS_GRIDMAPS_SERIALIZATION_STATIC_MAPS_GRIDMAP_HPP
#define CSLIBS_GRIDMAPS_SERIALIZATION_STATIC_MAPS_GRIDMAP_HPP

#include <cslibs_gridmaps/static_maps/gridmap.hpp>
#include <yaml-cpp/yaml.h>

namespace YAML {
template <typename T>
struct convert<std::shared_ptr<cslibs_gridmaps::static_maps::Gridmap<T>>>
{
    static Node encode(const typename cslibs_gridmaps::static_maps::Gridmap<T>::Ptr &rhs)
    {
        Node n;
        if (!rhs)
            return n;

        n.push_back(rhs->getOrigin());
        n.push_back(rhs->getResolution());
        n.push_back(rhs->getHeight());
        n.push_back(rhs->getWidth());
        n.push_back(rhs->getData());

        return n;
    }

    static bool decode(const Node& n, typename cslibs_gridmaps::static_maps::Gridmap<T>::Ptr &rhs)
    {
        if (!n.IsSequence() || n.size() != 5)
            return false;

        const std::size_t height = n[2].as<std::size_t>();
        const std::size_t width  = n[3].as<std::size_t>();
        rhs.reset(new cslibs_gridmaps::static_maps::Gridmap<T>(
                    n[0].as<cslibs_math_2d::Pose2d>(), n[1].as<double>(), height, width, -1.0));

        std::vector<T> data = n[4].as<std::vector<T>>();
        if (data.size() != height * width)
            return false;

        for (std::size_t p = 0 ; p < (width * height); ++ p)
            rhs->at(p) = data[p];

        return true;
    }
};
}

#endif // CSLIBS_GRIDMAPS_SERIALIZATION_STATIC_MAPS_GRIDMAP_HPP
