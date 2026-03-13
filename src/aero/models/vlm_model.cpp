#include "vlm_model.hpp"
#include <algorithm>

namespace aero {

    AeroOutput VLMModel::compute(const AeroInput& input) {
        AeroOutput out;
        return out;
    }

    void VLMModel::solve(Vector<VLMPanel>& panels, const Vector3& wind_velocity) {
        int N = static_cast<int>(panels.size());
        if (N == 0) return;

        Vector<Real> A(N * N, 0.0);
        Vector<Real> b(N, 0.0);
        Vector<Real> x(N, 0.0);

        build_influence_matrix(panels, wind_velocity, A, b);
        solve_linear_system(A, b, x);

        for (int i = 0; i < N; i++) {
            panels[i].circulation = x[i];
        }
    }

    void VLMModel::build_influence_matrix(const Vector<VLMPanel>& panels, const Vector3& wind_velocity, Vector<Real>& A, Vector<Real>& b) {
        int N = static_cast<int>(panels.size());
        Vector3 wind_dir = wind_velocity.normalized();

        Vector3 stable_wake_dir = Vector3(1, 0, 0);

        for (int i = 0; i < N; i++) {
            b[i] = -wind_velocity.dot(panels[i].normal);
            for (int j = 0; j < N; j++) {
                Vector3 v_unit = calculate_horseshoe_velocity(panels[i].collocation_point, panels[j], 1.0, wind_dir);
                A[i * N + j] = v_unit.dot(panels[i].normal);
            }
        }
    }

    void VLMModel::solve_linear_system(Vector<Real>& A, Vector<Real>& b, Vector<Real>& x) {
        int N = static_cast<int>(b.size());
        if (N == 0) return;

        // Row Scaling: Normalize rows by their largest element for better conditioning
        for (int i = 0; i < N; i++) {
            Real max_row_val = 0.0;
            for (int j = 0; j < N; j++) {
                max_row_val = std::max(max_row_val, std::abs(A[i * N + j]));
            }
            if (max_row_val > 1e-20) {
                Real scale = 1.0 / max_row_val;
                for (int j = 0; j < N; j++) A[i * N + j] *= scale;
                b[i] *= scale;
            }
        }

        // Gaussian Elimination with partial pivoting
        for (int i = 0; i < N; i++) {
            int pivot = i;
            Real max_val = std::abs(A[i * N + i]);
            for (int j = i + 1; j < N; j++) {
                Real val = std::abs(A[j * N + i]);
                if (val > max_val + 1e-15) { // Use epsilon to avoid unnecessary swaps
                    max_val = val;
                    pivot = j;
                }
            }
            if (pivot != i) {
                for (int k = 0; k < N; k++) std::swap(A[i * N + k], A[pivot * N + k]);
                std::swap(b[i], b[pivot]);
            }
            
            Real pivot_val = A[i * N + i];
            if (std::abs(pivot_val) < 1e-25) continue;

            for (int j = i + 1; j < N; j++) {
                Real factor = A[j * N + i] / pivot_val;
                b[j] -= factor * b[i];
                // Ensure zeros below diagonal for perfect numerical symmetry
                A[j * N + i] = 0.0; 
                for (int k = i + 1; k < N; k++) {
                    A[j * N + k] -= factor * A[i * N + k];
                }
            }
        }

        // Back substitution
        x.assign(N, 0.0);
        for (int i = N - 1; i >= 0; i--) {
            Real sum = 0;
            for (int j = i + 1; j < N; j++) sum += A[i * N + j] * x[j];
            if (std::abs(A[i * N + i]) > 1e-25) {
                x[i] = (b[i] - sum) / A[i * N + i];
            }
        }
    }

    // Inside vlm_model.cpp

// 1. Ensure this matches the header exactly
Vector3 VLMModel::calculate_induced_velocity(const Vector3& p, const Vector3& v1, const Vector3& v2, Real gamma, Real core_radius2) {
    Vector3 r1 = p - v1;
    Vector3 r2 = p - v2;
    Vector3 r1xr2 = r1.cross(r2);
    Real r1xr2_mag2 = r1xr2.length_squared();

    Real r1_mag = r1.length();
    Real r2_mag = r2.length();
    if (r1_mag < 1e-9 || r2_mag < 1e-9) return Vector3();

    Vector3 r0 = v2 - v1;
    Real factor = (gamma / (4.0 * M_PI)) * (r0.dot(r1 / r1_mag - r2 / r2_mag));
    
    // VATISTAS n=2 model: 
    // This replaces the Rankine std::max logic. 
    // It creates a smooth bell-curve for velocity that prevents the 3.232 AR spike.
    Real denominator = std::sqrt(std::pow(r1xr2_mag2, 2) + std::pow(core_radius2, 2));
    
    return r1xr2 * (factor / denominator);
}

// 2. Update the caller
Vector3 VLMModel::calculate_horseshoe_velocity(const Vector3& p, const VLMPanel& panel, Real gamma, const Vector3& wind_dir) {
    Vector3 v_total;
    
    Real panel_span = panel.area / panel.chord;
    // Use 10% of the span as the core radius for stability in sweep
    // Use the larger of a percentage of span or a fixed minimum 
    // to prevent the core from vanishing on high-density meshes.
    Real core_radius = std::max(panel_span * 0.05, 0.001);
    Real core_radius2 = core_radius * core_radius;

    // Call with the new 5th argument
    v_total = v_total + calculate_induced_velocity(p, panel.right_tip, panel.left_tip, gamma, core_radius2);

    auto semi_inf = [&](const Vector3& P1, const Vector3& D, Real G) {
        Vector3 r1 = p - P1;
        Vector3 Dxr1 = D.cross(r1);
        Real Dxr1_mag2 = Dxr1.length_squared();
        Real r1_mag = r1.length();
        
        if (r1_mag < 1e-10) return Vector3();
        
        return Dxr1 * ((G / (4.0 * M_PI)) * (1.0 + D.dot(r1) / r1_mag) / (Dxr1_mag2 + core_radius2));
    };

    v_total = v_total + semi_inf(panel.left_tip, wind_dir, -gamma);
    v_total = v_total + semi_inf(panel.right_tip, wind_dir, gamma);
    
    return v_total;
}

} // namespace aero
