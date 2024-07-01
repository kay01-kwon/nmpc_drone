#ifndef CONVEX_FN_HPP_
#define CONVEX_FN_HPP_
#include "type_definitions.hpp"

class Convex_fn{

    public:

    Convex_fn();
    Convex_fn(const mat31_t& bound,
    const double& epsilon);

    void get_fn_value(const mat31_t& vec,
    double& f, mat31_t& Df) const;

    void print_parameter_value();

    ~Convex_fn();

    private:

        double bound_scalar_;
        double epsilon_;

};

#endif