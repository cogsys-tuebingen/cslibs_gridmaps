#ifndef CSLIBS_GRIDMAPS_SERIALIZATION_DYNAMIC_MAPS_GRIDMAP_HPP
#define CSLIBS_GRIDMAPS_SERIALIZATION_DYNAMIC_MAPS_GRIDMAP_HPP

#include <cslibs_gridmaps/serialization/dynamic_maps/chunk.hpp>
#include <cslibs_gridmaps/dynamic_maps/gridmap.hpp>
#include <yaml-cpp/yaml.h>

namespace YAML {
template <>
struct convert<cslibs_math_2d::Transform2d>
{
    static Node encode(const cslibs_math_2d::Transform2d &rhs)
    {
        Node n;

        n.push_back(rhs.tx());
        n.push_back(rhs.ty());
        n.push_back(rhs.yaw());

        return n;
    }

    static bool decode(const Node& n, cslibs_math_2d::Transform2d &rhs)
    {
        if (!n.IsSequence() || n.size() != 3)
            return false;

        rhs.setFrom(n[0].as<double>(), n[1].as<double>(), n[2].as<double>());
        return true;
    }
};

template <typename T>
struct convert<std::shared_ptr<cslibs_gridmaps::dynamic_maps::Gridmap<T>>>
{
    static Node encode(const typename cslibs_gridmaps::dynamic_maps::Gridmap<T>::Ptr &rhs)
    {
        Node n;
        if (!rhs)
            return n;

        n.push_back(rhs->getInitialOrigin());
        n.push_back(rhs->getResolution());
        n.push_back(rhs->getChunkSize());
        n.push_back(rhs->getDefaultValue());

        std::array<int, 2> min_chunk_index = rhs->getMinChunkIndex();
        std::array<int, 2> max_chunk_index = rhs->getMaxChunkIndex();

        for (int idx = min_chunk_index[0] ; idx <= max_chunk_index[0] ; ++ idx) {
            for (int idy = min_chunk_index[1] ; idy <= max_chunk_index[1] ; ++ idy) {
                std::array<int, 2> index({idx, idy});
                if (const typename cslibs_gridmaps::dynamic_maps::Chunk<T>::handle_t c = rhs->getChunk(index)) {
                    cslibs_gridmaps::dynamic_maps::IndexedChunk<T, 2> ci(index, *c);
                    n.push_back(ci);
                }
            }
        }

        return n;
    }

    static bool decode(const Node& n, typename cslibs_gridmaps::dynamic_maps::Gridmap<T>::Ptr &rhs)
    {
        if (!n.IsSequence() || n.size() < 4)
            return false;

        const double resolution       = n[1].as<double>();
        const double chunk_resolution = resolution * (static_cast<double>(n[2].as<std::size_t>()) + 0.5);
        rhs.reset(new cslibs_gridmaps::dynamic_maps::Gridmap<T>(
                    n[0].as<cslibs_math_2d::Pose2d>(), resolution, chunk_resolution, n[3].as<double>()));

        for (std::size_t p = 4 ; p < n.size() ; ++ p) {
            cslibs_gridmaps::dynamic_maps::IndexedChunk<T, 2> ci =
                    n[p].as<cslibs_gridmaps::dynamic_maps::IndexedChunk<T, 2>>();
            rhs->getAllocateChunk(ci.index_).data() = ci.chunk_;
        }

        return true;
    }
};
}

#endif // CSLIBS_GRIDMAPS_SERIALIZATION_DYNAMIC_MAPS_GRIDMAP_HPP
