#include "convex_fn.hpp"

Convex_fn::Convex_fn()
{
    mat31_t bound;
    bound << 1, 1, 1;
    bound_scalar_ = sqrt(bound.transpose()*bound);
    epsilon_ = 1;
    
    cout << "Value for bound: " << endl;

    for(int i = 0 ; i < 3; i++)
        cout << bound(i) << endl;
    

    cout << "Value for epsilon: " << epsilon_ <<endl;

}

Convex_fn::Convex_fn(const mat31_t &bound, 
const double &epsilon):epsilon_(epsilon), 
bound_scalar_(sqrt(bound.transpose()*bound))
{
    cout << "Value for epsilon: " << epsilon_ <<endl;
}

void Convex_fn::get_fn_value(const mat31_t &vec, 
double &f, mat31_t &Df) const
{
    double num, den;

    num = vec.transpose()*vec 
    - pow(bound_scalar_,2.0);
    
    den = 2*epsilon_ *bound_scalar_
    + pow(epsilon_,2.0);

    f = num/den;
    
    Df = 2*vec/den;
}
