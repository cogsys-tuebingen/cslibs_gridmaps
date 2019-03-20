#ifndef CSLIBS_GRIDMAPS_INVERSE_MODEL_HPP
#define CSLIBS_GRIDMAPS_INVERSE_MODEL_HPP

#include <cslibs_math/common/log_odds.hpp>

#include <memory>

namespace cslibs_gridmaps {
namespace utility {
template <typename T>
class InverseModel
{
public:
    using Ptr = std::shared_ptr<InverseModel<T>>;

    inline InverseModel(const T prob_prior,
                        const T prob_free,
                        const T prob_occupied) :
        l_prior_(cslibs_math::common::LogOdds<T>::to(prob_prior)),
        l_free_(cslibs_math::common::LogOdds<T>::to(prob_free)),
        l_occupied_(cslibs_math::common::LogOdds<T>::to(prob_occupied))
    {
    }

    inline T getProbPrior() const
    {
        return cslibs_math::common::LogOdds<T>::from(l_prior_);
    }

    inline T getProbFree() const
    {
        return cslibs_math::common::LogOdds<T>::from(l_free_);
    }

    inline T getProbOccupied() const
    {
        return cslibs_math::common::LogOdds<T>::from(l_occupied_);
    }

    inline T getLogOddsPrior() const
    {
        return l_prior_;
    }

    inline T getLogOddsFree() const
    {
        return l_free_;
    }

    inline T getLogOddsOccupied() const
    {
        return l_occupied_;
    }

    inline T updateFree(const T l_rec) const
    {
        return l_free_ + l_rec - l_prior_;
    }

    inline T updateOccupied(const T l_rec) const
    {
        return l_occupied_ + l_rec - l_prior_;
    }

    // for weighted stuff
    inline T updateFree(const T l_rec, const int num, const T weight) const
    {
        return weight * l_free_ + l_rec - num * l_prior_;
    }

    inline T updateOccupied(const T l_rec, const int num, const T weight) const
    {
        return weight * l_occupied_ + l_rec - num * l_prior_;
    }

private:
    const T l_prior_;
    const T l_free_;
    const T l_occupied_;
};
}
}

#endif // CSLIBS_GRIDMAPS_INVERSE_MODEL_HPP
