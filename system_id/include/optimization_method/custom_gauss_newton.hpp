#ifndef CUSTOM_GAUSS_NEWTON_HPP
#define CUSTOM_GAUSS_NEWTON_HPP
#include "optimization_method.hpp"

template <typename Param>
class GaussNewton: public OptMethod<Param>{

    public:

        GaussNewton();

        virtual void minimize(Param &param_est);

        ~GaussNewton();

    private:

};

#endif

template <typename Param>
inline GaussNewton<Param>::GaussNewton()
{
    
}

template <typename Param>
inline void GaussNewton<Param>::minimize(Param &param_est)
{
    
}

template <typename Param>
inline GaussNewton<Param>::~GaussNewton()
{
}
