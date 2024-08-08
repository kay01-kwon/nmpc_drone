#ifndef OPTIMIZATION_METHOD_HPP
#define OPTIMIZATION_METHOD_HPP
#include <iostream>

#pragma once

using std::cout;
using std::endl;

enum class OptMethodName{GradDesc, GaussNewton, LevenbergMarquardt};

template <typename Param>
class OptMethod{

    public:

    OptMethod();

    virtual void minimize(Param &param_est) = 0;

    ~OptMethod();

    private:

};





#endif

template <typename Param>
inline OptMethod<Param>::OptMethod()
{
    cout << "OptMethod class" << endl;
}

template <typename Param>
inline OptMethod<Param>::~OptMethod()
{
}