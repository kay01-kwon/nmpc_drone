#include "gamma_proj.hpp"

GammaPrj::GammaPrj()
:Gamma_(1000*Gamma_.setIdentity())
{
}

GammaPrj::GammaPrj(const mat33_t &Gamma)
:Gamma_(Gamma)
{
}

void GammaPrj::getProjGamma(const mat31_t &vec, 
const mat31_t &y, 
const double &f, 
const mat31_t &grad_f, 
mat31_t &vec_proj)
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
