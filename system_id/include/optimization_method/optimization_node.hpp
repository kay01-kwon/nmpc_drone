#ifndef OPTIMIZATION_NODE_HPP
#define OPTIMIZATION_NODE_HPP

#include "optimization_method.hpp"
#include "custom_grad_descent.hpp"
#include "custom_gauss_newton.hpp"
#include "custom_levenberg_marquardt.hpp"
#include <memory>

template <typename Param>
class OptNode{
    
    public:

    OptNode();

    OptNode(const OptMethodName& method_name);

    private:

    std::unique_ptr< OptMethod<Param> > opt_method_ptr;

};

#endif

template <typename Param>
inline OptNode<Param>::OptNode(const OptMethodName &method_name)
{
    cout << "OptNode class" << endl;


    switch (method_name)
    {
    case OptMethodName::GradDesc:
        /* code */
        opt_method_ptr  = std::make_unique< GradDesc<Param> >();
        break;
    
    case OptMethodName::GaussNewton:

        opt_method_ptr = std::make_unique< GaussNewton<Param> >();
        break;

    case OptMethodName::LevenbergMarquardt:
        
        opt_method_ptr = std::make_unique< LevenbergMarquardt<Param> >();
        break;

    default:

        cout << "There is no available method at all." << endl;
        break;
    }

}