#include "dist_est/hgdo_dist_est.h"

HGDO::HGDO()
    : converter_(nullptr)
{
    gamma_tau_.setZero();
    gamma_f_.setZero();
    disturbance_.setZero();
}