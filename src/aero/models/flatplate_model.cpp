#include "flatplate_model.hpp"
#include <cmath>
#include <algorithm>

namespace aero {

AeroOutput FlatPlateModel::compute(const AeroInput& in) {
    AeroOutput out;
    if (in.speed < 0.1) return out;

    Real cl = 2.0 * M_PI * in.alpha;
    Real cd = 0.02 + (cl * cl) / (M_PI * 1.0); // Simple drag polar
    
    Real q = 0.5 * in.rho * in.speed * in.speed;
    out.lift = cl * q * in.area;
    out.drag = cd * q * in.area;

    // Convert lift/drag to local force vector (assuming alpha is in XY plane)
    // Lift is perpendicular to velocity, drag is parallel
    // Simple 2D approximation for now
    out.force = Vector3(0, out.lift, -out.drag); 

    return out;
}

} // namespace aero
