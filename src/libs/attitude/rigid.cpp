#include "rigid.h"
#include "attitude.h"

Eigen::Matrix<double, 7, 1> dynamics(const Eigen::Matrix<double, 7, 1> x, const double I[3][3], const double tau[3])
{
  Eigen::Matrix<double, 7, 1> x_dot;

  // Kinematics
  const double w[4] = {0, x(4), x(5), x(6)};
  const double q[4] = {x(0), x(1), x(2), x(3)};
  double q_dot[4];

  quat_prod(q, w, q_dot);
  x_dot(0) = 0.5f * q_dot[0];
  x_dot(1) = 0.5f * q_dot[1];
  x_dot(2) = 0.5f * q_dot[2];
  x_dot(3) = 0.5f * q_dot[3];

  // Kinetics
  Eigen::Matrix3d I_;
  Eigen::Vector3d Iw;
  Eigen::Vector3d w_(x[4], x[5], x[6]);
  Eigen::Vector3d tau_(tau[0], tau[1], tau[2]);
  I_ << I[0][0], I[0][1], I[0][2], I[1][0], I[1][1], I[1][2], I[2][0], I[2][1], I[2][2];
  Eigen::Vector3d w_dot = I_.inverse() * ((-w_.cross(I_ * w_)) + tau_);

  // Pack derivatives
  x_dot(4) = w_dot(0);
  x_dot(5) = w_dot(1);
  x_dot(6) = w_dot(2);

  return x_dot;
}

state_t rk4(const state_t x, const double dt, const double I[3][3], const double tau[3])
{
  Eigen::Matrix<double, 7, 1> y(x.q[0], x.q[1], x.q[2], x.q[3], x.w[0], x.w[1], x.w[2]);
  Eigen::Matrix<double, 7, 1> k1 = dynamics(y, I, tau);
  Eigen::Matrix<double, 7, 1> k2 = dynamics(y + 0.5 * dt * k1, I, tau);
  Eigen::Matrix<double, 7, 1> k3 = dynamics(y + 0.5 * dt * k2, I, tau);
  Eigen::Matrix<double, 7, 1> k4 = dynamics(y + dt * k3, I, tau);
  Eigen::Matrix<double, 7, 1> K = (1 / 6.0f) * (k1 + 2 * k2 + 2 * k3 + k4);
  Eigen::Matrix<double, 7, 1> y_new = y + K * dt;

  state_t x_new;
  x_new.q[0] = y_new[0];
  x_new.q[1] = y_new[1];
  x_new.q[2] = y_new[2];
  x_new.q[3] = y_new[3];
  x_new.w[0] = y_new[4];
  x_new.w[1] = y_new[5];
  x_new.w[2] = y_new[6];

  return x_new;
}
