#ifndef LOW_PASS_FILTER_HPP_
#define LOW_PASS_FILTER_HPP_
#include "tools.hpp"
#include <boost/lambda/lambda.hpp>
#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/external/eigen/eigen.hpp>

using boost::numeric::odeint::runge_kutta4;

class Lpf{

    public:

    Lpf() = delete;

    Lpf(const double& tau);

    void set_input(const mat31_t& v_in);

    void set_time(const double& t);

    void get_filtered_vector(mat31_t& v_out);

    private:

    // vector to be processed
    mat31_t curr_v_, v_in_, v_out_;

    // time constant
    double tau_;

    // Time info
    double curr_time_, prev_time_, dt_;

    /**
     * Declare rk dopri5 class
    */
    runge_kutta4<mat31_t> rk4;

    void operator()(
        const mat31_t& v,
        mat31_t& dvdt,
        double t);

    void solve();
};


#endif