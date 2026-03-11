#pragma once

#include "aero/core/aero_model.hpp"

namespace aero {

struct VLMPanel {
    Vector3 left_tip;
    Vector3 right_tip;
    Vector3 collocation_point;
    Vector3 normal;
    Real area;
    Real chord;
    Real circulation = 0.0;
};

class VLMModel : public IAeroModel {
public:
    VLMModel() = default;

    // IAeroModel interface
    AeroOutput compute(const AeroInput& input) override;

    // VLM specific: solve for multiple panels
    static void solve(Vector<VLMPanel>& panels, const Vector3& wind_velocity);

private:
    static Vector3 _calculate_induced_velocity(const Vector3& p, const Vector3& v1, const Vector3& v2, Real gamma);
    static Vector3 _calculate_horseshoe_velocity(const Vector3& p, const VLMPanel& panel, Real gamma, const Vector3& wind_dir);
};

} // namespace aero
