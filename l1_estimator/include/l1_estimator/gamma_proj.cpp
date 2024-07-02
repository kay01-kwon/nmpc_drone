#include "gamma_proj.hpp"

GammaPrj::GammaPrj(const mat33_t &Gamma)
:Gamma_(Gamma)
{
    print_Gamma_value();
}

void GammaPrj::getProjGamma(const mat31_t &vec, 
const mat31_t &y, 
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

void GammaPrj::print_Gamma_value() const
{
    cout << "Value for Gamma:" << endl;
    cout << Gamma_ <<endl;
}

GammaPrj::~GammaPrj()
{
    
}
