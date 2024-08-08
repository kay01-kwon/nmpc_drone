#ifndef CUSTOM_LEVENBERG_MARQUARDT_HPP
#define CUSTOM_LEVENBERG_MARQUARDT_HPP
#include "optimization_method.hpp"

template <typename Param>
class LevenbergMarquardt: public OptMethod<Param>{

    public:

    LevenbergMarquardt();

    virtual void minimize(Param &param_est);

    ~LevenbergMarquardt();


    private:

};


#endif

template <typename Param>
inline LevenbergMarquardt<Param>::LevenbergMarquardt()
{
}

template <typename Param>
inline void LevenbergMarquardt<Param>::minimize(Param &param_est)
{
}

template <typename Param>
inline LevenbergMarquardt<Param>::~LevenbergMarquardt()
{
}
