#include "low_pass_filter.hpp"


Lpf::Lpf(const double &tau)
    : tau_(tau), curr_v_(curr_v_.setZero()),
      v_in_(v_in_.setZero()), v_out_(v_out_.setZero()),
      curr_time_(0), prev_time_(0), dt_(0)
{
    cout<<"Tau set: "<<tau_<<endl;
}

void Lpf::set_input_and_time(const mat31_t &v_in, const double& t)
{
    v_in_ = v_in;
    curr_time_ = t;
}

void Lpf::get_filtered_vector(mat31_t &v_out)
{
    solve();
    v_out = v_out_;
}

void Lpf::operator()(const mat31_t &v, 
mat31_t &dvdt, double t)
{
    dvdt = -tau_*v + tau_*v_in_;
}

void Lpf::solve()
{
    dt_ = curr_time_ - prev_time_;

    rk4.do_step([this] 
    (const mat31_t& v, mat31_t& dvdt, double t)
    {
        this->operator()(v, dvdt, t);
    },
    v_out_, prev_time_, dt_);
    prev_time_ = curr_time_;
}
