#include "gamma_proj.hpp"

/**
 * @brief Construct a new Gamma Prj:: Gamma Prj object
 * 
 * @param Gamma Gain for Gamma projection (3 by 3 matrix)
 */
GammaPrj::GammaPrj(const mat33_t &Gamma)
:Gamma_(Gamma)
{
    print_Gamma_value();
}

/**
 * @brief Project y into the gradient of its convex function
 * to prevent parameter drift
 * 
 * @param y 
 * @param f 
 * @param grad_f 
 * @param vec_proj 
 */
void GammaPrj::getProjGamma(const mat31_t &y, 
const double &f, 
const mat31_t &grad_f, 
mat31_t &vec_proj) const
{
    if( (f > 0) && ( y.transpose()*Gamma_*grad_f > 0 ))
    {
        vec_proj = Gamma_*(
            y - 
            grad_f*grad_f.transpose()/(grad_f.transpose()*grad_f)
            *Gamma_*y*f
        );
    }
    else
    {
        vec_proj = Gamma_*y;
    }
}

/**
 * @brief Print out the gain for gamma projection
 * 
 */
void GammaPrj::print_Gamma_value() const
{
    cout << "Value for Gamma:" << endl;
    cout << Gamma_ <<endl;
}

GammaPrj::~GammaPrj()
{
    
}