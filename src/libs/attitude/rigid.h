#ifndef _SIMDEZVOUS_RIGID_H_
#define _SIMDEZVOUS_RIGID_H_

#include <Eigen>

typedef struct
{
  double q[4], w[3];
} state_t;

Eigen::Matrix<double, 7, 1> dynamics(const Eigen::Matrix<double, 7, 1> x, const double I[3][3], const double tau[3]);
state_t rk4(const state_t x, const double dt, const double I[3][3], const double tau[3]);

#endif // rigid.h

