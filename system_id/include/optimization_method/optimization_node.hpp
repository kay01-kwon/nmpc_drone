#ifndef OPTIMIZATION_NODE_HPP
#define OPTIMIZATION_NODE_HPP

#include "optimization_method.hpp"
#include "custom_grad_descent.hpp"
#include <memory>

template <typename Meas>
class OptNode{
    
    public:

    OptNode();

    OptNode(const OptMethodName& method_name);

    private:

    std::unique_ptr< OptMethod<Meas> > opt_method_ptr;

};

#endif

template <typename Meas>
inline OptNode<Meas>::OptNode(const OptMethodName &method_name)
{
    cout << "OptNode class" << endl;

    if(method_name == OptMethodName::GradDesc)
    {
        opt_method_ptr = std::make_unique< GradDesc<Meas> >();
    }
}