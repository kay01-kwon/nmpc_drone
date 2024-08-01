#ifndef ODE_RK4_CUSTOM_H_
#define ODE_RK4_CUSTOM_H_

#include <iostream>


template<typename StateInOut>
class OdeRK4Custom{

    public:

        OdeRK4Custom();

        template<typename System>
        void do_step(System system_dynamics, 
        StateInOut& s, const double &prev_time, 
        const double& dt);

    private:

    double a1, a2, a3, a4;

};

#endif

template <typename StateInOut>
inline OdeRK4Custom<StateInOut>::OdeRK4Custom()
:a1(static_cast<double>(1.0)), 
a2(static_cast<double>(2.0)), 
a3(static_cast<double>(2.0)), 
a4(static_cast<double>(1.0))
{
}

template <typename StateInOut>
template <typename System>
inline void OdeRK4Custom<StateInOut>::do_step(System system_dynamics, 
StateInOut &s, const double &prev_time, 
const double &dt)
{
    StateInOut K1, K2, K3, K4;

    StateInOut s_temp;

    system_dynamics(s, K1, prev_time);

    s_temp = s + 0.5*dt*K1;
    system_dynamics(s_temp, K2 ,prev_time + 0.5*dt);

    s_temp = s + 0.5*dt*K2;
    system_dynamics(s_temp, K3, prev_time + 0.5*dt);

    s_temp = s + dt*K3;
    system_dynamics(s_temp, K4, prev_time + dt);

    s += (K1*a1 + K2*a2 + K3*a3 + K4*a4)/static_cast<double>(6) * dt;
}