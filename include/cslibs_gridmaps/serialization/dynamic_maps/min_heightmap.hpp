#ifndef CSLIBS_GRIDMAPS_SERIALIZATION_DYNAMIC_MAPS_MIN_HEIGHTMAP_HPP
#define CSLIBS_GRIDMAPS_SERIALIZATION_DYNAMIC_MAPS_MIN_HEIGHTMAP_HPP

#include <cslibs_gridmaps/serialization/dynamic_maps/gridmap.hpp>
#include <cslibs_gridmaps/dynamic_maps/min_heightmap.h>
#include <yaml-cpp/yaml.h>

namespace YAML {
template <typename Tp, typename T>
struct convert<std::shared_ptr<cslibs_gridmaps::dynamic_maps::MinHeightmap<Tp, T>>>
{
    static Node encode(const typename cslibs_gridmaps::dynamic_maps::MinHeightmap<Tp, T>::Ptr &rhs)
    {
        Node n;
        if (!rhs)
            return n;

        n.push_back(rhs->getInitialOrigin());
        n.push_back(rhs->getResolution());
        n.push_back(rhs->getChunkSize());
        n.push_back(rhs->getMaxHeight());
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

    static bool decode(const Node& n, typename cslibs_gridmaps::dynamic_maps::MinHeightmap<Tp, T>::Ptr &rhs)
    {
        if (!n.IsSequence() || n.size() < 4)
            return false;

        const Tp resolution       = n[1].as<Tp>();
        const Tp chunk_resolution = resolution * (static_cast<Tp>(n[2].as<std::size_t>()) + 0.5);
        rhs.reset(new cslibs_gridmaps::dynamic_maps::MinHeightmap<Tp, T>(
                    n[0].as<cslibs_math_2d::Pose2d<Tp>>(), resolution, chunk_resolution, n[3].as<T>(), n[4].as<double>()));

        for (std::size_t p = 5 ; p < n.size() ; ++ p) {
            cslibs_gridmaps::dynamic_maps::IndexedChunk<T, 2> ci =
                    n[p].as<cslibs_gridmaps::dynamic_maps::IndexedChunk<T, 2>>();
            rhs->getAllocateChunk(ci.index_).data() = ci.chunk_;
        }

        return true;
    }
};
}

#endif // CSLIBS_GRIDMAPS_SERIALIZATION_DYNAMIC_MAPS_MIN_HEIGHTMAP_HPP
