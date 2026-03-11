#pragma once

#include "aero_math.hpp"

namespace aero {

struct AeroInput {
    Real rho = 1.225;
    Real speed = 0.0;
    Real alpha = 0.0; // rad
    Real beta = 0.0;  // rad
    Real area = 1.0;
    Real chord = 1.0;
    Vector3 wind_velocity;
};

struct AeroOutput {
    Vector3 force;    // Local force vector
    Vector3 moment;   // Local moment vector around reference point
    Real lift = 0.0;
    Real drag = 0.0;
};

class IAeroModel {
public:
    virtual ~IAeroModel() = default;
    virtual AeroOutput compute(const AeroInput& input) = 0;
};

} // namespace aero
