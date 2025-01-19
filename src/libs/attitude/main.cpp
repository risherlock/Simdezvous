// #include <math.h>
// #include <vector>
// #include <fstream>
// #include <iostream>

// #include <Eigen>
// #include "rigid.h"
// #include "attitude.h"

// #define D2R 0.01745329251
// #define R2D 57.2957795131

// int sign(double x)
// {
//   if(x<0)
//   {
//     return -1;
//   }

//   return 1;
// }

// // tau = -kp * sign(dq(4)) * dq(1 : 3) - kd * w;
// void control_law(const state_t err, const double kp, const double kd, double tau[3])
// {
//   int q0_sign = sign(err.q[0]);
//   tau[0] = -kp * q0_sign * err.q[1] - kd * err.w[0];
//   tau[1] = -kp * q0_sign * err.q[2] - kd * err.w[1];
//   tau[2] = -kp * q0_sign * err.q[3] - kd * err.w[2];
// }

// state_t get_err(const state_t xd, const state_t xf)
// {
//   state_t xerr;

//   quat_err(xd.q, xf.q, xerr.q);
//   xerr.w[0] = xf.w[0];
//   xerr.w[1] = xf.w[1];
//   xerr.w[2] = xf.w[2];

//   return xerr;
// }

// int main()
// {
//   // Simulation params [s]
//   double stop_time = 300;
//   double dt = 1;
//   const size_t n = stop_time / dt;

//   // Physical Parameters [kg m^2]
//   const double I[3][3] = {{10000, 0, 0}, {0, 9000, 0}, {0, 0, 12000}};

//   // Control gains
//   double kp = 50;
//   double kd = 500;

//   // Initial state
//   state_t x0;
//   x0.q[0] = 0.153;
//   x0.q[1] = 0.685;
//   x0.q[2] = 0.695;
//   x0.q[3] = 0.153;
//   x0.w[0] = -0.53 * D2R;
//   x0.w[1] = 0.53 * D2R;
//   x0.w[2] = 0.053 * D2R;

//   // Desired state
//   state_t xd;
//   xd.q[0] = 1.0f;
//   xd.q[1] = 0.0f;
//   xd.q[2] = 0.0f;
//   xd.q[3] = 0.0f;
//   xd.w[0] = 0.0f;
//   xd.w[1] = 0.0f;
//   xd.w[2] = 0.0f;

//   std::vector<state_t> x;
//   x.push_back(x0);

//   std::ofstream file;
//   file.open("output.csv");

//    file << double(0) << ","
//          << x0.q[0] << "," << x0.q[1] << "," << x0.q[2] << "," << x0.q[3] << ","
//          << x0.w[0] << "," << x0.w[1] << "," << x0.w[2] << std::endl;

//   for (size_t i = 0; i < n; i++)
//   {
//     state_t xerr = get_err(xd, x[i]);
   
//     double tau[3];
//     control_law(xerr, kp, kd, tau);

//     x.push_back(rk4(x[i], dt, I, tau));
//     quat_normalize(x[i+1].q);

//     file << double((i+1) * dt) << ","
//          << x[i+1].q[0] << "," << x[i+1].q[1] << "," << x[i+1].q[2] << "," << x[i+1].q[3] << ","
//          << x[i+1].w[0] << "," << x[i+1].w[1] << "," << x[i+1].w[2] << std::endl;
//   }
// }

