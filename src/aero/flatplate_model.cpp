#include "flatplate_model.hpp"
#include <cmath>

static float cl_flatplate(float alpha)
{
    return 2.0f * sin(alpha) * cos(alpha);
}

static float cd_flatplate(float alpha)
{
    float s = sin(alpha);
    return 2.0f * s * s;
}

static float cm_flatplate(float cl)
{
    return -0.25f * cl;
}

AeroOutput flatplate_compute(const AeroInput& in)
{
    AeroOutput out{};

    float q = 0.5f * in.rho * in.speed * in.speed;

    float cl = cl_flatplate(in.alpha);
    float cd = cd_flatplate(in.alpha);
    float cm = cm_flatplate(cl);

    out.lift   = q * in.area * cl;
    out.drag   = q * in.area * cd;
    out.moment = q * in.area * in.chord * cm;

    return out;
}
