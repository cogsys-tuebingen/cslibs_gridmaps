#ifndef CSLIBS_GRIDMAPS_SERIALIZATION_STATIC_MAPS_DISTANCE_GRIDMAP_HPP
#define CSLIBS_GRIDMAPS_SERIALIZATION_STATIC_MAPS_DISTANCE_GRIDMAP_HPP

#include <cslibs_gridmaps/serialization/aligned_vector.hpp>
#include <cslibs_gridmaps/serialization/static_maps/gridmap.hpp>
#include <cslibs_gridmaps/static_maps/distance_gridmap.h>
#include <yaml-cpp/yaml.h>

namespace YAML {
template<typename Tp, typename T, typename AllocatorT>
struct convert<std::shared_ptr<cslibs_gridmaps::static_maps::DistanceGridmap<Tp,T,AllocatorT>>>
{
    static Node encode(const typename cslibs_gridmaps::static_maps::DistanceGridmap<Tp,T,AllocatorT>::Ptr &rhs)
    {
        Node n;
        if (!rhs)
            return n;

        n.push_back(rhs->getOrigin());
        n.push_back(rhs->getResolution());
        n.push_back(rhs->getMaximumDistance());
        n.push_back(rhs->getHeight());
        n.push_back(rhs->getWidth());
        n.push_back(rhs->getData());

        return n;
    }

    static bool decode(const Node& n, typename cslibs_gridmaps::static_maps::DistanceGridmap<Tp,T,AllocatorT>::Ptr &rhs)
    {
        if (!n.IsSequence() || n.size() != 6)
            return false;

        const std::size_t height = n[3].as<std::size_t>();
        const std::size_t width  = n[4].as<std::size_t>();
        rhs.reset(new cslibs_gridmaps::static_maps::DistanceGridmap<Tp,T,AllocatorT>(
                    n[0].as<cslibs_math_2d::Pose2<Tp>>(), n[1].as<Tp>(), n[2].as<T>(), height, width));

        std::vector<T,AllocatorT> data = n[5].as<std::vector<T,AllocatorT>>();
        if (data.size() != height * width)
            return false;

        for (std::size_t p = 0 ; p < (width * height); ++ p)
            rhs->at(p) = data[p];

        return true;
    }
};
}

#endif // CSLIBS_GRIDMAPS_SERIALIZATION_STATIC_MAPS_DISTANCE_GRIDMAP_HPP
