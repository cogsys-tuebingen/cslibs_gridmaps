#ifndef CSLIBS_GRIDMAPS_STATIC_GRIDMAP_HPP
#define CSLIBS_GRIDMAPS_STATIC_GRIDMAP_HPP

#include <array>
#include <vector>
#include <cmath>
#include <memory>

#include <cslibs_math_2d/linear/pose.hpp>
#include <cslibs_math_2d/linear/point.hpp>

#include <cslibs_gridmaps/static_maps/algorithms/bresenham.hpp>
#include <cslibs_gridmaps/static_maps/algorithms/bresenham_unsafe.hpp>

namespace cslibs_gridmaps {
namespace static_maps {
template<typename Tp, typename T, typename AllocatorT = std::allocator<T>> // Tp for accuracy of transforms etc., T for content
class EIGEN_ALIGN16 Gridmap
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using allocator_t = Eigen::aligned_allocator<Gridmap<Tp, T, AllocatorT>>;

    using Ptr                    = std::shared_ptr<Gridmap<Tp, T, AllocatorT>>;
    using const_line_iterator_t  = algorithms::Bresenham<T const>;
    using index_t                = std::array<int, 2>;
    using pose_t                 = cslibs_math_2d::Pose2<Tp>;
    using point_t                = cslibs_math_2d::Point2<Tp>;

    inline Gridmap(const pose_t &origin,
                   const Tp resolution,
                   const std::size_t height,
                   const std::size_t width,
                   const T &default_value) :
        resolution_(resolution),
        resolution_inv_(1.0 / resolution),
        height_(height),
        width_(width),
        max_index_({{static_cast<int>(width)-1,static_cast<int>(height)-1}}),
        w_T_m_(origin),
        m_T_w_(w_T_m_.inverse()),
        data_(height * width, default_value),
        data_ptr_(data_.data())
    {
    }

    inline Gridmap(const Tp origin_x,
                   const Tp origin_y,
                   const Tp origin_phi,
                   const Tp resolution,
                   const std::size_t height,
                   const std::size_t width,
                   const T &default_value) :
        resolution_(resolution),
        resolution_inv_(1.0 / resolution),
        height_(height),
        width_(width),
        max_index_({{static_cast<int>(width)-1,static_cast<int>(height)-1}}),
        w_T_m_(origin_x, origin_y, origin_phi),
        m_T_w_(w_T_m_.inverse()),
        data_(height * width, default_value),
        data_ptr_(data_.data())
    {
    }

    inline Gridmap(const Gridmap &other) :
        resolution_(other.resolution_),
        resolution_inv_(other.resolution_inv_),
        height_(other.height_),
        width_(other.width_),
        max_index_(other.max_index_),
        w_T_m_(other.w_T_m_),
        m_T_w_(other.m_T_w_),
        data_(other.data_),
        data_ptr_(data_.data())
    {
    }

    inline Gridmap(Gridmap &&other) :
        resolution_(other.resolution_),
        resolution_inv_(other.resolution_inv_),
        height_(other.height_),
        width_(other.width_),
        max_index_(other.max_index_),
        w_T_m_(other.w_T_m_),
        m_T_w_(other.m_T_w_),
        data_(other.data_),
        data_ptr_(data_.data())
    {
    }

    virtual ~Gridmap()
    {
    }

    /**
     * @brief Get the minimum in map coordinates.
     * @return the minimum
     */
    virtual inline point_t getMin() const
    {
        return point_t();
    }

    /**
     * @brief Get the maximum in map coordinates.
     * @return the maximum
     */
    virtual inline point_t getMax() const
    {
        return point_t(width_ *resolution_, height_ * resolution_);
    }

    /**
     * @brief Get the origin
     * @return the origin
     */
    virtual inline pose_t getOrigin() const
    {
        return w_T_m_;
    }

    inline T& at(const std::size_t idx, const std::size_t idy)
    {
        return data_ptr_[width_ * idy + idx];
    }

    inline const T& at(const std::size_t idx, const std::size_t idy) const
    {
        return data_ptr_[width_ * idy + idx];
    }

    inline T& at(const std::size_t i)
    {
        return data_ptr_[i];
    }

    inline const T& at(const std::size_t i) const
    {
        return data_ptr_[i];
    }

    virtual inline T& at(const point_t &point)
    {
        index_t i;
        if(!toIndex(point, i)) {
            throw std::runtime_error("[GridMap] : Invalid Index!");
        }
        return at(i[0], i[1]);
    }

    virtual inline T at(const point_t &point) const
    {
        index_t i;
        if(!toIndex(point, i)) {
            throw std::runtime_error("[GridMap] : Invalid Index!");
        }
        return at(i[0], i[1]);
    }

    inline const_line_iterator_t getConstLineIterator(const index_t &start,
                                                      const index_t &end) const
    {
        /// do index capping
        if(invalid(start)) {
            throw std::runtime_error("[GridMap]: Start index is invalid!");
        }
        if(invalid(end)) {
            throw std::runtime_error("[GridMap]: End index is invalid!");
        }
        return const_line_iterator_t(start, end, width_, data_ptr_);
    }

    inline const_line_iterator_t getConstLineIterator(const point_t &start,
                                                      const point_t &end) const
    {
        index_t start_index;
        index_t end_index;
        toIndex(start, start_index);
        toIndex(end, end_index);
        return const_line_iterator_t(start_index,
                                     end_index,
                                     {static_cast<int>(width_), static_cast<int>(height_)},
                                     data_ptr_);
    }

    inline Tp getResolution() const
    {
        return resolution_;
    }

    inline std::size_t getHeight() const
    {
        return height_;
    }

    inline std::size_t getWidth() const
    {
        return width_;
    }

    inline index_t getMaxIndex() const
    {
        return max_index_;
    }

    std::vector<T, AllocatorT> & getData()
    {
        return data_;
    }

    std::vector<T, AllocatorT> const & getData() const
    {
        return data_;
    }

    inline bool toIndex(const point_t &p_w,
                        index_t &i) const
    {
        const point_t p_m = m_T_w_ * p_w;

        i[0] = static_cast<int>(std::floor(p_m(0) * resolution_inv_));
        i[1] = static_cast<int>(std::floor(p_m(1) * resolution_inv_));

        return (i[0] >= 0 && i[0] <= max_index_[0]) &&
               (i[1] >= 0 && i[1] <= max_index_[1]);
    }

    inline virtual bool validate(const pose_t &p_w) const
    {
      const point_t p_m = m_T_w_ * p_w.translation();
      index_t i = {{static_cast<int>(std::floor(p_m(0) * resolution_inv_)),
                    static_cast<int>(std::floor(p_m(1) * resolution_inv_))}};

      return (i[0] >= 0 && i[0] <= max_index_[0]) &&
             (i[1] >= 0 && i[1] <= max_index_[1]);
    }

protected:
    const Tp            resolution_;        /// m / cell
    const Tp            resolution_inv_;    /// cell / m
    const std::size_t   height_;
    const std::size_t   width_;
    const index_t       max_index_;
    const pose_t        w_T_m_;
    const pose_t        m_T_w_;

    std::vector<T, AllocatorT>  data_;
    T*                  	data_ptr_;

    inline bool invalid(const index_t &_i) const
    {
        return _i[0] < 0 ||
               _i[1] < 0 ||
               _i[0] > max_index_[0] ||
               _i[1] > max_index_[1];
    }

    inline void fromIndex(const index_t &i,
                          point_t &p_w) const
    {
        p_w = w_T_m_ * point_t(static_cast<Tp>(i[0]) * resolution_,
                               static_cast<Tp>(i[1]) * resolution_);
    }

    inline void fromIndex(const const_line_iterator_t &it,
                          point_t &p_w) const
    {
        p_w = w_T_m_ * point_t(static_cast<Tp>(it.x()) * resolution_,
                               static_cast<Tp>(it.y()) * resolution_);
    }
};
}
}

#endif // CSLIBS_GRIDMAPS_STATIC_GRIDMAP_HPP
