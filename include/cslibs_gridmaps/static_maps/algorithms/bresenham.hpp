#ifndef CSLIBS_GRIDMAPS_STATIC_BRESENHAM_HPP
#define CSLIBS_GRIDMAPS_STATIC_BRESENHAM_HPP

#include <cslibs_math_2d/algorithms/bresenham.hpp>

namespace cslibs_gridmaps {
namespace static_maps {
namespace algorithms {
template<typename T>
class Bresenham
{
public:
    using index_t = std::array<int, 2>;
    using size_t  = std::array<int, 2>;

    inline explicit Bresenham(const index_t &start,
                              const index_t &end,
                              const size_t   size,
                              T *            data) :
        bresenham_(start, end),
        data_(data),
        size_(size)
    {
    }

    inline int x() const
    {
        return bresenham_.x();
    }

    inline int y() const
    {
        return bresenham_.y();
    }

    inline Bresenham& operator++()
    {
        ++bresenham_;
        return *this;
    }

    inline bool iterate()
    {
        ++bresenham_;
        return !(done() || invalid());
    }

    inline bool done() const
    {
        return bresenham_.done();
    }

    inline bool invalid() const
    {
        return  bresenham_.x() <= -1 ||
                bresenham_.y() <= -1 ||
                bresenham_.x() >= size_[0] ||
                bresenham_.y() >= size_[1];
    }

    inline T operator *() const
    {
        return data_[bresenham_.y() * size_[0] + bresenham_.x()];
    }

    inline T& operator *()
    {
        return data_[bresenham_.y() * size_[0] + bresenham_.x()];
    }

private:
    cslibs_math_2d::algorithms::Bresenham bresenham_;
    T                                    *data_;
    size_t                                size_;
};
}
}
}

#endif // CSLIBS_GRIDMAPS_STATIC_BRESENHAM_HPP
