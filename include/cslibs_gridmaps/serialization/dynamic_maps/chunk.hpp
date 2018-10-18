#ifndef CSLIBS_GRIDMAPS_SERIALIZATION_DYNAMIC_MAPS_CHUNK_HPP
#define CSLIBS_GRIDMAPS_SERIALIZATION_DYNAMIC_MAPS_CHUNK_HPP

#include <cslibs_gridmaps/dynamic_maps/chunk.hpp>
#include <cslibs_math/serialization/array.hpp>
#include <yaml-cpp/yaml.h>

namespace cslibs_gridmaps {
namespace dynamic_maps {
template <typename T, std::size_t Dim>
struct IndexedChunk {
    std::array<int, Dim> index_;
    Chunk<T>             chunk_;

    IndexedChunk() = default;

    IndexedChunk(
            const std::array<int, Dim> & index,
            const Chunk<T>             & chunk) :
        index_(index),
        chunk_(chunk)
    {
    }
};
}
}

namespace YAML {
template <typename T>
struct convert<cslibs_gridmaps::dynamic_maps::Chunk<T>>
{
    static Node encode(const cslibs_gridmaps::dynamic_maps::Chunk<T> &rhs)
    {
        Node n;

        n.push_back(rhs.size());
        n.push_back(rhs.getData());

        return n;
    }

    static bool decode(const Node& n, cslibs_gridmaps::dynamic_maps::Chunk<T> &rhs)
    {
        if (!n.IsSequence() || n.size() != 2)
            return false;

        const int size = n[0].as<int>();
        rhs = cslibs_gridmaps::dynamic_maps::Chunk<T>(size, T());

        std::vector<T> data = n[1].as<std::vector<T>>();
        if (data.size() != static_cast<std::size_t>(size * size))
            return false;

        for (int idx = 0 ; idx < size ; ++ idx)
            for (int idy = 0 ; idy < size ; ++ idy)
                rhs.at(idx, idy) = data[idy * size + idx];

        return true;
    }
};

template <typename T, std::size_t Dim>
struct convert<cslibs_gridmaps::dynamic_maps::IndexedChunk<T, Dim>>
{
    static Node encode(const cslibs_gridmaps::dynamic_maps::IndexedChunk<T, Dim> &rhs)
    {
        Node n;

        n.push_back(rhs.index_);
        n.push_back(rhs.chunk_);

        return n;
    }

    static bool decode(const Node& n, cslibs_gridmaps::dynamic_maps::IndexedChunk<T, Dim> &rhs)
    {
        if (!n.IsSequence() || n.size() != 2)
            return false;

        rhs.index_ = n[0].as<std::array<int, Dim>>();
        rhs.chunk_ = n[1].as<cslibs_gridmaps::dynamic_maps::Chunk<T>>();

        return true;
    }
};
}

#endif // CSLIBS_GRIDMAPS_SERIALIZATION_DYNAMIC_MAPS_CHUNK_HPP
