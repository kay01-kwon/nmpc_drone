#ifndef LOW_PASS_FILTER_HPP_
#define LOW_PASS_FILTER_HPP_
#include "tools.hpp"
#include "ode_solver/ode_include.h"

class Lpf{

    public:

    Lpf() = delete;

    Lpf(const double& tau);

    void set_input_and_time(const mat31_t& v_in, const double& t);

    void get_filtered_vector(mat31_t& v_out) const;

    void solve();

    ~Lpf() = default;

    private:

    // vector to be processed
    mat31_t curr_v_, v_in_, v_out_;

    // time constant
    double tau_;

    // Time info
    double curr_time_, prev_time_, dt_;

    /**
     * Declare runge kutta class
    */
    runge_kutta4_classic<mat31_t> rk4_classic_;
    runge_kutta4<mat31_t> rk4_;
    runge_kutta_dopri5<mat31_t> rk45_;

    void system_dynamics(const mat31_t& v,
        mat31_t& dvdt,
        const double& t);


};

#endif