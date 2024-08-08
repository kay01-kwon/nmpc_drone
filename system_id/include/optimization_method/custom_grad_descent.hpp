#ifndef CUSTOM_GRAD_DESCENT_HPP
#define CUSTOM_GRAD_DESCENT_HPP
#include "optimization_method.hpp"

template <typename functor>
class GradDesc{

    public:
        GradDesc();

    private:


};


#endif

template <typename functor>
inline GradDesc<functor>::GradDesc()
{
    cout << "GradDes class" << endl;
}
