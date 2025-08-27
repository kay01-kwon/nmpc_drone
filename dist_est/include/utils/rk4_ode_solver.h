#ifndef RK4_ODE_SOLVER_H
#define RK4_ODE_SOLVER_H

#include <iostream>

template <typename StateInOut>
class Rk4OdeSolver{

    public:

    Rk4OdeSolver();

    template <typename SystemDynamics>
    void do_step(SystemDynamics system_dynamics,
                StateInOut &s,
                const double &t_prev,
                const double &dt);

    private:

};

#endif

template <typename StateInOut>
Rk4OdeSolver<StateInOut>::Rk4OdeSolver()
{
}

template <typename StateInOut>
template <typename SystemDynamics>
void Rk4OdeSolver<StateInOut>::do_step(SystemDynamics system_dynamics,
                                      StateInOut &s,
                                      const double &t_prev,
                                      const double &dt)
{
    StateInOut k1, k2, k3, k4;
    
    // 1. First stage
    system_dynamics(s, k1, t_prev);

    // 2. Second stage
    StateInOut s_temp = s + 0.5 * dt * k1;
    system_dynamics(s_temp, k2, t_prev + 0.5 * dt);


    // 3. Third stage
    s_temp = s + 0.5 * dt * k2;
    system_dynamics(s_temp, k3, t_prev + 0.5 * dt);

    // 4. Fourth stage
    s_temp = s + dt * k3;
    system_dynamics(s_temp, k4, t_prev + dt);

    s += (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);

}