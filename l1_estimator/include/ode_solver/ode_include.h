#ifndef ODE_INCLUDE_H_
#define ODE_INCLUDE_H_

#include <boost/lambda/lambda.hpp>
#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/external/eigen/eigen.hpp>

using boost::numeric::odeint::runge_kutta4;
using boost::numeric::odeint::runge_kutta_dopri5;
using boost::numeric::odeint::runge_kutta_cash_karp54;
using boost::numeric::odeint::runge_kutta_fehlberg78;

using boost::numeric::odeint::integrate_adaptive;
using boost::numeric::odeint::integrate_const;
using boost::numeric::odeint::make_dense_output;
using boost::numeric::odeint::make_controlled;


#endif