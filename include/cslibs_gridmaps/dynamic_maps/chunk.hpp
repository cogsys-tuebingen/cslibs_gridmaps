#ifndef CSLIBS_GRIDMAPS_DYNAMIC_CHUNK_HPP
#define CSLIBS_GRIDMAPS_DYNAMIC_CHUNK_HPP

#include <vector>
#include <array>
#include <mutex>
#include <atomic>

namespace cslibs_gridmaps {
namespace dynamic_maps {
template<typename T>
class Chunk {
public:
    using index_t   = std::array<int, 2>;
    using mutex_t   = std::mutex;
    using lock_t    = std::unique_lock<mutex_t>;

    enum Action {NONE, ALLOCATED, TOUCHED};

    inline Chunk() :
        action_(ALLOCATED)
    {
    }

    virtual ~Chunk() = default;

    inline Chunk(const int size,
          const T default_value) :
        size_(size),
        data_(size * size, default_value),
        data_ptr_(data_.data())
    {
    }

    inline Chunk(const Chunk &other) :
        size_(other.size_),
        data_(other.data_),
        data_ptr_(data_.data())
    {
    }

    inline Chunk(Chunk &&other) :
        size_(other.size_),
        data_(std::move(other.data_)),
        data_ptr_(data_.data())
    {
    }

    inline Chunk& operator = (const Chunk &other)
    {
        size_  = (other.size_);
        data_  = (other.data_);
        data_ptr_ = (data_.data());
        return *this;
    }

    inline Chunk& operator = (Chunk &&other)
    {
        size_  =          (other.size_);
        data_  = std::move(other.data_);
        data_ptr_ =       (data_.data());
        return *this;
    }

    inline void setTouched()
    {
        if(action_ != ALLOCATED)
            action_ = TOUCHED;
    }

    inline void setNone()
    {
        action_ = NONE;
    }

    inline Action getAction() const
    {
        return action_;
    }

    inline T const & at(const index_t &i) const
    {
        return data_ptr_[i[1] * size_ + i[0]];
    }

    inline T & at (const index_t &i)
    {
        return data_ptr_[i[1] * size_ + i[0]];
    }

    inline T & at (const int idx, const int idy)
    {
        return data_ptr_[idy * size_ + idx];
    }

    inline T const & at (const int idx, const int idy) const
    {
        return data_ptr_[idy * size_ + idx];
    }

    inline void merge(const Chunk &)
    {
    }

    inline void lock() const
    {
        data_mutex_.lock();
    }

    inline void unlock() const
    {
        data_mutex_.unlock();
    }

private:
    Action              action_;
    int                 size_;
    std::vector<T>      data_;
    T                  *data_ptr_;
    mutable mutex_t     data_mutex_;

};
}
}


#endif // CSLIBS_GRIDMAPS_DYNAMIC_CHUNK_HPP
