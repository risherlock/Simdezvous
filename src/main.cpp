#include <iostream>
#include <Eigen>
#include "attitude.h"
#include "rigid.h"

#define D2R 0.01745329251
#define R2D 57.2957795131

void controlLaw()
{
}

state_t get_err(const state_t xd, const state_t xf)
{
    state_t xerr;

    quat_err(xd.q, xf.q, xerr.q);
    xerr.w[0] = xf.w[0];
    xerr.w[1] = xf.w[1];
    xerr.w[2] = xf.w[2];

    return xerr;
}

int main()
{
    // Simulation Parameters
    double stop_time = 300;
    double sampling_time = 1;
    const size_t n = stop_time / sampling_time;

    // Physical Parameters [kg m^2]
    const double I[3][3] = {{10000, 0, 0}, {0, 9000, 0}, {0, 0, 12000}};

    // Control gains
    double kp = 50;
    double kd = 500;

    // Initial state
    state_t x0;
    x0.q[0] = 0.153;
    x0.q[1] = 0.685;
    x0.q[2] = 0.695;
    x0.q[3] = 0.153;
    x0.w[0] = -0.53 * D2R;
    x0.w[1] = 0.53 * D2R;
    x0.w[2] = 0.053 * D2R;

    // Magnetic Field Vector
    // 221B Baker Street
    const float latitude = 51.5238;  // deg
    const float longitude = -0.1586; // deg
    const float height = 1000.0;     // km
    const float x_sph[3] = {latitude, longitude, height};

    // Time
    date_time dt;
    dt.year = 2023;
    dt.month = 12;
    dt.day = 17;
    dt.hour = 0;
    dt.minute = 0;
    dt.second = 0;

    const double magnetic_vector;
    igrf(dt, x_sph, magnetic_vector);

    // steps:
    // describe vector as quaternion
    // convert a quaternion from earth's frame to frame of satellite
    // find rotation matrix from current rotation matrix

    // Desired state
    state_t xd;

    quat_axis_angle(0, magnetic_vector, xd.q);

    xd.w[0] = 0.0f;
    xd.w[1] = 0.0f;
    xd.w[2] = 0.0f;


}