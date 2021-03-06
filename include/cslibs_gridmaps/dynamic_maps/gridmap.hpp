#ifndef CSLIBS_GRIDMAPS_DYNAMIC_GRIDMAP_HPP
#define CSLIBS_GRIDMAPS_DYNAMIC_GRIDMAP_HPP

#include <array>
#include <vector>
#include <cmath>
#include <memory>

#include <cslibs_math_2d/linear/pose.hpp>
#include <cslibs_math_2d/linear/point.hpp>

#include <cslibs_gridmaps/dynamic_maps/algorithms/bresenham.hpp>
#include <cslibs_gridmaps/dynamic_maps/chunk.hpp>

#include <cslibs_math/common/array.hpp>
#include <cslibs_math/common/mod.hpp>
#include <cslibs_math/common/div.hpp>

#include <cslibs_indexed_storage/storage.hpp>
#include <cslibs_indexed_storage/backend/kdtree/kdtree.hpp>

#include <cslibs_utility/common/delegate.hpp>

namespace cis = cslibs_indexed_storage;

namespace cslibs_gridmaps {
namespace dynamic_maps {
template<typename Tp, typename T, typename AllocatorT = std::allocator<T>> // Tp for accuracy of transforms etc., T for content
class EIGEN_ALIGN16 Gridmap
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<Gridmap<Tp, T, AllocatorT>>;

    using Ptr                   = std::shared_ptr<Gridmap<Tp, T, AllocatorT>>;
    using gridmap_t             = Gridmap<Tp, T, AllocatorT>;
    using point_t               = cslibs_math_2d::Point2<Tp>;
    using pose_t                = cslibs_math_2d::Pose2<Tp>;
    using index_t               = std::array<int, 2>;
    using mutex_t               = std::mutex;
    using lock_t                = std::unique_lock<mutex_t>;
    using chunk_t               = Chunk<T, AllocatorT>;
    using storage_t             = cis::Storage<chunk_t, index_t, cis::backend::kdtree::KDTree>;
    using line_iterator_t       = algorithms::Bresenham<T, AllocatorT>;
    using const_line_iterator_t = algorithms::Bresenham<T const, AllocatorT>;
    using get_chunk_t           = cslibs_utility::common::delegate<typename chunk_t::handle_t(const index_t &)>;

    inline Gridmap(const pose_t &origin,
                   const Tp      resolution,
                   const Tp      chunk_resolution,
                   const T      &default_value) :
        resolution_(resolution),
        resolution_inv_(1.0 / resolution_),
        chunk_size_(static_cast<int>(chunk_resolution * resolution_inv_)),
        default_value_(default_value),
        w_T_m_(origin),
        m_T_w_(w_T_m_.inverse()),
        min_chunk_index_{{std::numeric_limits<int>::max(), std::numeric_limits<int>::max()}},
        max_chunk_index_{{std::numeric_limits<int>::min(), std::numeric_limits<int>::min()}},
        min_index_{{std::numeric_limits<int>::max(), std::numeric_limits<int>::max()}},
        storage_(new storage_t),
        height_(chunk_size_),
        width_(chunk_size_)
    {
    }

    inline Gridmap(const Tp origin_x,
                   const Tp origin_y,
                   const Tp origin_phi,
                   const Tp resolution,
                   const Tp chunk_resolution,
                   const T &default_value) :
        resolution_(resolution),
        resolution_inv_(1.0 / resolution_),
        chunk_size_(static_cast<int>(chunk_resolution * resolution_inv_)),
        default_value_(default_value),
        w_T_m_(origin_x, origin_y, origin_phi),
        m_T_w_(w_T_m_.inverse()),
        min_chunk_index_{{std::numeric_limits<int>::max(), std::numeric_limits<int>::max()}},
        max_chunk_index_{{std::numeric_limits<int>::min(), std::numeric_limits<int>::min()}},
        min_index_{{std::numeric_limits<int>::max(), std::numeric_limits<int>::max()}},
        storage_(new storage_t),
        height_(chunk_size_),
        width_(chunk_size_)
    {
    }

    inline Gridmap(const Gridmap &other) :
        resolution_(other.resolution_),
        resolution_inv_(other.resolution_inv_),
        chunk_size_(other.chunk_size_),
        default_value_(other.default_value_),
        w_T_m_(other.w_T_m_),
        m_T_w_(other.m_T_w_),
        min_chunk_index_(other.min_chunk_index_),
        max_chunk_index_(other.max_chunk_index_),
        min_index_(other.min_index_),
        storage_(new storage_t(*other.storage_)),
        height_(other.chunk_size_),
        width_(other.chunk_size_)
    {
    }


    inline Gridmap(Gridmap &&other) :
        resolution_(other.resolution_),
        resolution_inv_(other.resolution_inv_),
        chunk_size_(other.chunk_size_),
        default_value_(other.default_value_),
        w_T_m_(std::move(other.w_T_m_)),
        m_T_w_(std::move(other.m_T_w_)),
        min_chunk_index_(other.min_chunk_index_),
        max_chunk_index_(other.max_chunk_index_),
        min_index_(other.min_index_),
        storage_(new storage_t(*other.storage_)),
        height_(other.chunk_size_),
        width_(other.chunk_size_)
    {
    }

    virtual ~Gridmap() = default;

    /**
     * @brief Get minimum in map coordinates.
     * @return the minimum
     */
    inline point_t getMin() const
    {
        lock_t l(storage_mutex_);
        return point_t(min_chunk_index_[0] * chunk_size_ * resolution_,
                       min_chunk_index_[1] * chunk_size_ * resolution_);

    }

    /**
     * @brief Get maximum in map coordinates.
     * @return the maximum
     */
    inline point_t getMax() const
    {
        lock_t l(storage_mutex_);
        return point_t((max_chunk_index_[0] + 1) * chunk_size_ * resolution_,
                       (max_chunk_index_[1] + 1) * chunk_size_ * resolution_);
    }

    /**
     * @brief Get the current origin of the map.
     * @return the origin
     */
    inline pose_t getOrigin() const
    {
        pose_t origin = w_T_m_;
        origin.translation() = getMin();
        return origin;
    }

    /**
     * @brief Get the initial origin of the map.
     * @return  the intial origin
     */
    inline pose_t getInitialOrigin() const
    {
        return w_T_m_;
    }

    inline T get(const std::size_t idx, const std::size_t idy) const
    {
        index_t index;
        {
            lock_t l(storage_mutex_);
            if (idx >= width_ || idy >= height_) {
                throw std::runtime_error("[GridMap] : Invalid Index!");
            }

            index = {static_cast<int>(idx) + min_index_[0],
                     static_cast<int>(idy) + min_index_[1]};
        }
        const index_t chunk_index       = toChunkIndex(index);
        const index_t local_chunk_index = toLocalChunkIndex(index);

        const typename chunk_t::const_handle_t chunk = getChunk(chunk_index);
        return chunk.empty() ? default_value_ : chunk->at(local_chunk_index);
    }

    inline void set(const point_t &point,
                    const T &v)
    {
        const index_t index              = toIndex(point);
        const index_t chunk_index        = toChunkIndex(index);
        const index_t local_chunk_index  = toLocalChunkIndex(index);

        typename chunk_t::handle_t chunk = getAllocateChunk(chunk_index);
        chunk->at(local_chunk_index)     = v;
    }

    inline T get(const point_t &point) const
    {
        const index_t index             = toIndex(point);
        const index_t chunk_index       = toChunkIndex(index);
        const index_t local_chunk_index = toLocalChunkIndex(index);

        const typename chunk_t::const_handle_t chunk = getChunk(chunk_index);
        return chunk.empty() ? default_value_ : chunk->at(local_chunk_index);
    }

    inline line_iterator_t getLineIterator(const index_t &start_index,
                                           const index_t &end_index)
    {
        return line_iterator_t(start_index,
                               end_index,
                               chunk_size_,
                               get_chunk_t::template from<gridmap_t, &gridmap_t::getAllocateChunk>(this));
    }

    inline line_iterator_t getLineIterator(const point_t &start,
                                           const point_t &end)
    {

        const index_t start_index = toIndex(start);
        const index_t end_index   = toIndex(end);
        return line_iterator_t(start_index,
                               end_index,
                               chunk_size_,
                               get_chunk_t::template from<gridmap_t, &gridmap_t::getAllocateChunk>(this));
    }

    inline line_iterator_t getLineIterator(const index_t &start_index,
                                           const index_t &end_index) const
    {
        return line_iterator_t(start_index,
                               end_index,
                               chunk_size_,
                               get_chunk_t::template from<gridmap_t, &gridmap_t::getAllocateChunk>(this));
    }

    inline line_iterator_t getLineIterator(const point_t &start,
                                           const point_t &end) const
    {
        const index_t start_index = toIndex(start);
        const index_t end_index   = toIndex(end);
        return line_iterator_t(start_index,
                               end_index,
                               chunk_size_,
                               get_chunk_t::template from<gridmap_t, &gridmap_t::getAllocateChunk>(this));
    }

    inline index_t getMinChunkIndex() const
    {
        lock_t l(storage_mutex_);
        return min_chunk_index_;
    }

    inline index_t getMaxChunkIndex() const
    {
        lock_t l(storage_mutex_);
        return max_chunk_index_;
    }

    inline typename chunk_t::const_handle_t const getChunk(const index_t &chunk_index) const
    {
        lock_t l(storage_mutex_);
        const chunk_t *c = storage_->get(chunk_index);
        return c ? c->getHandle() : typename chunk_t::const_handle_t();
    }

    inline typename chunk_t::handle_t getChunk(const index_t &chunk_index)
    {
        lock_t l(storage_mutex_);
        chunk_t *c = storage_->get(chunk_index);
        return c ? c->getHandle() : typename chunk_t::handle_t();
    }

    inline typename chunk_t::handle_t getAllocateChunk(const index_t &chunk_index) const
    {
        lock_t l(storage_mutex_);
        chunk_t *chunk = storage_->get(chunk_index);
        if (chunk == nullptr) {
            chunk = &(storage_->insert(chunk_index, chunk_t(chunk_size_, default_value_)));
        }
        updateChunkIndices(chunk_index);
        return chunk->getHandle();
    }

    inline Tp getResolution() const
    {
        return resolution_;
    }

    inline std::size_t getChunkSize() const
    {
        return static_cast<std::size_t>(chunk_size_);
    }

    inline std::size_t getHeight() const
    {
        lock_t l(storage_mutex_);
        return height_;
    }

    inline std::size_t getWidth() const
    {
        lock_t l(storage_mutex_);
        return width_;
    }

    inline index_t getMaxIndex() const
    {
        lock_t l(storage_mutex_);
        return {(max_chunk_index_[0] - min_chunk_index_[0] + 1) * chunk_size_ - 1,
                (max_chunk_index_[1] - min_chunk_index_[1] + 1) * chunk_size_ - 1};
    }

    inline T getDefaultValue() const
    {
        return default_value_;
    }

    inline virtual bool validate(const pose_t &p_w) const
    {
        index_t i = toChunkIndex(toIndex(p_w.translation()));
        return i[0] >= min_chunk_index_[0] && i[0] <= max_chunk_index_[0] &&
               i[1] >= min_chunk_index_[1] && i[1] <= max_chunk_index_[1];
    }

protected:
    const Tp                           resolution_;
    const Tp                           resolution_inv_;
    const int                          chunk_size_;
    const T                            default_value_;
    pose_t                             w_T_m_;
    pose_t                             m_T_w_;

    mutable index_t                    min_chunk_index_;
    mutable index_t                    max_chunk_index_;
    mutable index_t                    min_index_;
    mutable mutex_t                    storage_mutex_;
    mutable std::unique_ptr<storage_t> storage_;
    mutable std::size_t                height_;
    mutable std::size_t                width_;

    inline void updateChunkIndices(const index_t &chunk_index) const
    {
        min_chunk_index_    = std::min(min_chunk_index_, chunk_index);
        max_chunk_index_    = std::max(max_chunk_index_, chunk_index);
        min_index_          = min_chunk_index_ * chunk_size_;
        width_  = (max_chunk_index_[0] - min_chunk_index_[0] + 1) * chunk_size_;
        height_ = (max_chunk_index_[1] - min_chunk_index_[1] + 1) * chunk_size_;
    }

    inline index_t toChunkIndex(const index_t &index) const
    {
        return {{cslibs_math::common::div(index[0], chunk_size_),
                 cslibs_math::common::div(index[1], chunk_size_)}};
    }

    inline index_t toLocalChunkIndex(const index_t &index) const
    {
        return {{cslibs_math::common::mod(index[0], chunk_size_),
                 cslibs_math::common::mod(index[1], chunk_size_)}};
    }

    inline index_t toIndex(const point_t &p_w) const
    {
        /// offset and rounding correction!
        const point_t p_m = m_T_w_ * p_w;
        return {{static_cast<int>(std::floor(p_m(0) * resolution_inv_)),
                 static_cast<int>(std::floor(p_m(1) * resolution_inv_))}};
    }
};
}
}

#endif // CSLIBS_GRIDMAPS_DYNAMIC_GRIDMAP_HPP
