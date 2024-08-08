#ifndef OPTIMIZATION_METHOD_HPP
#define OPTIMIZATION_METHOD_HPP
#include <iostream>

#pragma once

using std::cout;
using std::endl;

enum class OptMethodName{GradDes, GaussNewton, LM};

template <typename Meas>
class OptMethod{

    public:

    OptMethod();

    template <typename Param>
    virtual void minimize(Param &param_est) const;

    virtual ~OptMethod();

    private:

};





#endif

template <typename Meas>
inline OptMethod<Meas>::OptMethod()
{
    cout << "OptMethod class" << endl;
}

template <typename Meas>
template <typename Param>
inline void OptMethod<Meas>::minimize(Param &param_est) const
{
    cout << "Minimize function" << endl;
}
