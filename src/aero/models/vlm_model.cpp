#include "vlm_model.hpp"
#include <algorithm>

namespace aero {

AeroOutput VLMModel::compute(const AeroInput& input) {
    // This would be for a single element solver if needed, 
    // but VLM is usually global. For single element we fallback to simple lift/drag.
    AeroOutput out;
    return out;
}

void VLMModel::solve(Vector<VLMPanel>& panels, const Vector3& wind_velocity) {
    int N = static_cast<int>(panels.size());
    if (N == 0) return;

    Vector3 wind_dir = wind_velocity.normalized();
    Real wind_speed = wind_velocity.length();
    if (wind_speed < 0.001) return;

    Vector<Real> A(N * N, 0.0);
    Vector<Real> b(N, 0.0);

    for (int i = 0; i < N; i++) {
        b[i] = -wind_velocity.dot(panels[i].normal);
        for (int j = 0; j < N; j++) {
            Vector3 v_unit = _calculate_horseshoe_velocity(panels[i].collocation_point, panels[j], 1.0, wind_dir);
            A[i * N + j] = v_unit.dot(panels[i].normal);
        }
    }

    // Row Scaling
    for (int i = 0; i < N; i++) {
        Real diag = std::abs(A[i * N + i]);
        if (diag > 1e-15) {
            Real scale = 1.0 / A[i * N + i];
            for (int j = 0; j < N; j++) A[i * N + j] *= scale;
            b[i] *= scale;
        }
    }

    // Gaussian Elimination
    for (int i = 0; i < N; i++) {
        int pivot = i;
        Real max_val = std::abs(A[i * N + i]);
        for (int j = i + 1; j < N; j++) {
            Real val = std::abs(A[j * N + i]);
            if (val > max_val) { max_val = val; pivot = j; }
        }
        if (pivot != i) {
            for (int k = 0; k < N; k++) std::swap(A[i * N + k], A[pivot * N + k]);
            std::swap(b[i], b[pivot]);
        }
        if (std::abs(A[i * N + i]) < 1e-25) continue;
        for (int j = i + 1; j < N; j++) {
            Real factor = A[j * N + i] / A[i * N + i];
            b[j] -= factor * b[i];
            for (int k = i; k < N; k++) A[j * N + k] -= factor * A[i * N + k];
        }
    }

    Vector<Real> x(N, 0.0);
    for (int i = N - 1; i >= 0; i--) {
        Real sum = 0;
        for (int j = i + 1; j < N; j++) sum += A[i * N + j] * x[j];
        if (std::abs(A[i * N + i]) > 1e-25) x[i] = (b[i] - sum) / A[i * N + i];
    }

    for (int i = 0; i < N; i++) panels[i].circulation = x[i];
}

Vector3 VLMModel::_calculate_induced_velocity(const Vector3& p, const Vector3& v1, const Vector3& v2, Real gamma) {
    Vector3 r1 = p - v1;
    Vector3 r2 = p - v2;
    Vector3 r1xr2 = r1.cross(r2);
    Real r1xr2_mag2 = r1xr2.length_squared();

    const Real core_radius2 = 1e-8;
    Real r1_mag = r1.length();
    Real r2_mag = r2.length();
    if (r1_mag < 1e-12 || r2_mag < 1e-12) return Vector3();

    Vector3 r0 = v2 - v1;
    Real factor = (gamma / (4.0 * M_PI)) * (r0.dot(r1 / r1_mag - r2 / r2_mag));
    return r1xr2 * (factor / (r1xr2_mag2 + core_radius2));
}

Vector3 VLMModel::_calculate_horseshoe_velocity(const Vector3& p, const VLMPanel& panel, Real gamma, const Vector3& wind_dir) {
    Vector3 v_total;
    v_total += _calculate_induced_velocity(p, panel.left_tip, panel.right_tip, gamma);

    auto semi_inf = [&](const Vector3& P1, const Vector3& D, Real G) {
        Vector3 r1 = p - P1;
        Vector3 Dxr1 = D.cross(r1);
        Real Dxr1_mag2 = Dxr1.length_squared();
        Real r1_mag = r1.length();
        if (r1_mag < 1e-12) return Vector3();
        const Real core_radius2 = 1e-8;
        return Dxr1 * ((G / (4.0 * M_PI)) * (1.0 + D.dot(r1) / r1_mag) / (Dxr1_mag2 + core_radius2));
    };

    v_total += semi_inf(panel.left_tip, wind_dir, -gamma);
    v_total += semi_inf(panel.right_tip, wind_dir, gamma);
    return v_total;
}

} // namespace aero
