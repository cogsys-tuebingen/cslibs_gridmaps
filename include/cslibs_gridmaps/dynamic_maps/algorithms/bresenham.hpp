#ifndef CSLIBS_GRIDMAPS_DYNAMIC_BRESENHAM_HPP
#define CSLIBS_GRIDMAPS_DYNAMIC_BRESENHAM_HPP

#include <array>

#include <cslibs_utility/common/delegate.hpp>
#include <cslibs_gridmaps/dynamic_maps/chunk.hpp>

#include <cslibs_math/common/div.hpp>
#include <cslibs_math/common/mod.hpp>
#include <cslibs_math/common/array.hpp>

#include <cslibs_math_2d/algorithms/bresenham.hpp>

namespace cslibs_gridmaps {
namespace dynamic_maps {
namespace algorithms {
template<typename T>
class Bresenham
{
public:
    using Ptr           = std::shared_ptr<Bresenham>;
    using index_t       = std::array<int, 2>;
    using chunk_t       = dynamic_maps::Chunk<T>;
    using get_chunk_t   = cslibs_utility::common::delegate<typename chunk_t::handle_t(const index_t&)>;

    inline explicit Bresenham(const index_t     &start,
                              const index_t     &end,
                              const int          chunk_size,
                              const T           &default_value,
                              const get_chunk_t &get_chunk) :
        get_chunk_(get_chunk),
        chunk_size_(chunk_size),
        diff_{{(end[0] - start[0]), (end[1] - start[1])}},
        global_bresenham_(start, end),
        default_value_(default_value)
    {
        update();
    }

    inline virtual ~Bresenham()
    {
    }

    inline int x() const
    {
        return global_bresenham_.x();
    }

    inline int y() const
    {
        return global_bresenham_.y();
    }

    inline int lx() const
    {
        return local_bresenham_.x();
    }

    inline int ly() const
    {
        return local_bresenham_.y();
    }

    inline Bresenham& operator++()
    {
        if(global_bresenham_.done()) {
            active_chunk_ = typename chunk_t::handle_t();
            return *this;
        }

        ++global_bresenham_;
        ++local_bresenham_;

        if(localIndexInvalid()) {
            update();
        }

        return *this;
    }

    inline bool done() const
    {
        return global_bresenham_.done();
    }

    inline T & operator *()
    {
        assert(!active_chunk_.empty());
        return active_chunk_->at(lx(), ly());
    }

    inline T const & operator *() const
    {
        assert(!active_chunk_.empty());
        return active_chunk_->at(lx(), ly());
    }

private:
    get_chunk_t                 get_chunk_;
    typename chunk_t::handle_t  active_chunk_;
    int                         chunk_size_;
    index_t                     diff_;

    cslibs_math_2d::algorithms::Bresenham global_bresenham_;
    cslibs_math_2d::algorithms::Bresenham local_bresenham_;

    index_t      local_index_start_;
    index_t      local_index_end_;

    index_t      chunk_index_;
    T            default_value_;

    inline void update()
    {
        chunk_index_[0]  = cslibs_math::common::div(global_bresenham_.x(), chunk_size_);
        chunk_index_[1]  = cslibs_math::common::div(global_bresenham_.y(), chunk_size_);

        local_index_start_ = {{cslibs_math::common::mod(global_bresenham_.x(), chunk_size_),
                               cslibs_math::common::mod(global_bresenham_.y(), chunk_size_)}};
        local_index_end_   = local_index_start_ + diff_;

        local_bresenham_ = cslibs_math_2d::algorithms::Bresenham(local_index_start_,
                                                                 local_index_end_);

        active_chunk_ = get_chunk_(chunk_index_);
    }

    inline bool localIndexInvalid()
    {
        return local_bresenham_.x() <= -1 || local_bresenham_.x() >= chunk_size_ ||
               local_bresenham_.y() <= -1 || local_bresenham_.y() >= chunk_size_;
    }
};
}
}
}

#endif // CSLIBS_GRIDMAPS_BRESENHAM_HPP
