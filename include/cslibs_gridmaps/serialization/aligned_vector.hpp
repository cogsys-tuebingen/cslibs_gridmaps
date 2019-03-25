#ifndef CSLIBS_GRIDMAPS_SERIALIZATION_ALIGNED_VECTOR_HPP
#define CSLIBS_GRIDMAPS_SERIALIZATION_ALIGNED_VECTOR_HPP

#include <eigen3/Eigen/Core>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace YAML {
template <typename T>
struct convert<std::vector<T, Eigen::aligned_allocator<T>>>
{
    static Node encode(const std::vector<T, Eigen::aligned_allocator<T>>& rhs)
    {
        Node n(NodeType::Sequence);
        for (typename std::vector<T, Eigen::aligned_allocator<T>>::const_iterator it = rhs.begin(); it != rhs.end(); ++it)
            n.push_back(*it);
        return n;
    }

    static bool decode(const Node& n, std::vector<T, Eigen::aligned_allocator<T>>& rhs)
    {
        if (!n.IsSequence())
            return false;
        rhs.clear();
        for (const_iterator it = n.begin(); it != n.end(); ++it)
#if defined(__GNUC__) && __GNUC__ < 4
            // workaround for GCC 3:
            rhs.push_back(it->template as<T>());
#else
            rhs.push_back(it->as<T>());
#endif
        return true;
    }
};
}

#endif // CSLIBS_GRIDMAPS_SERIALIZATION_ALIGNED_VECTOR_HPP
