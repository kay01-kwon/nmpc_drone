#ifndef CUSTOM_GRAD_DESCENT_HPP
#define CUSTOM_GRAD_DESCENT_HPP
#include "optimization_method.hpp"

template <typename Param>
class GradDesc: public OptMethod<Param>{

    public:

    GradDesc();

    virtual void minimize(Param &param_est);

    ~GradDesc();

    private:


};


#endif

template <typename Param>
inline GradDesc<Param>::GradDesc()
{
    cout << "GradDes class" << endl;
}

template <typename Param>
inline void GradDesc<Param>::minimize(Param &param_est)
{
}

template <typename Meas>
inline GradDesc<Meas>::~GradDesc()
{
}