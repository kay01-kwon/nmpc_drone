#ifndef GAMMA_PROJ_HPP_
#define GAMMA_PROJ_HPP_
#include "type_definitions.hpp"

class GammaPrj{

    public:

        GammaPrj() = delete;

        GammaPrj(const mat33_t& Gamma);

        void getProjGamma(const mat31_t& y, 
        const double& f, 
        const mat31_t& grad_f,
        mat31_t& vec_proj) const;

        void print_Gamma_value() const;

        ~GammaPrj();

    private:

        mat33_t Gamma_;

};

#endif