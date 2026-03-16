#include "vlm_model.hpp"
#include <algorithm>
#include <cmath>

#include <cstdio>

namespace aero {

void VLMModel::solve(Vector<VLMPanel>& p_panels, const Vector3& wind_velocity, bool debug) {
    const int N = static_cast<int>(p_panels.size());
    if (N == 0) return;

    Vector<Real> A(N * N, 0.0);
    Vector<Real> b(N, 0.0);
    Vector<Real> x(N, 0.0);

    build_influence_matrix(p_panels, wind_velocity, A, b);
    
    if (debug) {
        printf("\n--- VLM Solver Debug ---\n");
        printf("N=%d, Wind=(%.2f, %.2f, %.2f)\n", N, wind_velocity.x, wind_velocity.y, wind_velocity.z);
        printf("Influence Matrix (A) [First 3 rows]:\n");
        for (int i = 0; i < std::min(3, N); i++) {
            printf("Row %d: ", i);
            for (int j = 0; j < std::min(3, N); j++) printf("%8.4f ", A[i * N + j]);
            if (N > 3) printf("...");
            printf(" | b[%d] = %8.4f\n", i, b[i]);
        }
    }

    // This call matches your void return type
    solve_linear_system(A, b, x);

    for (int i = 0; i < N; i++) {
        p_panels[i].circulation = x[i];
    }
}

void VLMModel::build_influence_matrix(const Vector<VLMPanel>& p_panels, const Vector3& wind_velocity, Vector<Real>& A, Vector<Real>& b) {
    const int N = static_cast<int>(p_panels.size());

    for (int i = 0; i < N; i++) {
        // RHS: Boundary condition must use the actual wind velocity
        b[i] = -wind_velocity.dot(p_panels[i].normal);

        for (int j = 0; j < N; j++) {
            // For stability at high AoA, we use the panel's chord direction for the wake
            // instead of the free-stream wind direction. This keeps the influence matrix
            // well-conditioned.
            Vector3 mid_bound = (p_panels[j].left_tip + p_panels[j].right_tip) * 0.5;
            Vector3 wake_dir = (p_panels[j].collocation_point - mid_bound).normalized();
            
            Vector3 v_unit = calculate_horseshoe_velocity(p_panels[i].collocation_point, p_panels[j], 1.0, wake_dir);
            A[i * N + j] = v_unit.dot(p_panels[i].normal);
        }
    }
}

Vector3 VLMModel::calculate_induced_velocity(const Vector3& p, const Vector3& v1, const Vector3& v2, Real gamma, Real eps2) {
    const Vector3 r1 = p - v1;
    const Vector3 r2 = p - v2;
    const Vector3 r1xr2 = r1.cross(r2);
    const Real r1xr2_mag2 = r1xr2.length_squared();

    const Real r1_mag = r1.length();
    const Real r2_mag = r2.length();

    // Singularity check
    if (r1_mag < 1e-12 || r2_mag < 1e-12) return Vector3(0, 0, 0);

    const Vector3 r0 = v2 - v1;
    const Real kernel = (gamma / (4.0 * M_PI));
    const Real dot_term = r0.dot(r1 / r1_mag - r2 / r2_mag);
    
    // Improved Vatistas / Modified Core radius
    // We use (r1xr2_mag2 + eps^2) where eps^2 has dimensions L^4
    const Real denominator = r1xr2_mag2 + eps2;
    
    return r1xr2 * (kernel * dot_term / denominator);
}

Vector3 VLMModel::calculate_horseshoe_velocity(const Vector3& p, const VLMPanel& panel, Real gamma, const Vector3& wind_dir) {
    Vector3 v_total(0, 0, 0);
    
    // Dimensional eps^4 core to match r1xr2_mag2 (|L^2|^2)
    // Using a fraction of the panel span as the core radius is more robust than a fixed minimum.
    Real panel_width = (panel.left_tip - panel.right_tip).length();
    const Real core_r = std::max(1e-6, 0.1 * panel_width);
    const Real eps2 = std::pow(core_r, 4); 

    // 1. Bound Vortex (finite segment)
    v_total = v_total + calculate_induced_velocity(p, panel.right_tip, panel.left_tip, gamma, eps2);

    // 2. Trailing Semi-Infinite Vortices
    auto semi_inf = [&](const Vector3& P1, const Vector3& D, Real G) {
        const Vector3 r1 = p - P1;
        const Vector3 Dxr1 = D.cross(r1);
        const Real Dxr1_mag2 = Dxr1.length_squared();
        const Real r1_mag = r1.length();
        
        if (r1_mag < 1e-12) return Vector3(0, 0, 0);
        
        // Denominator for semi-infinite must be dimensionally consistent (L^2)
        const Real eps_trailing2 = core_r * core_r; 
        return Dxr1 * ((G / (4.0 * M_PI)) * (1.0 + D.dot(r1) / r1_mag) / (Dxr1_mag2 + eps_trailing2));
    };

    v_total = v_total + semi_inf(panel.left_tip, wind_dir, gamma);
    v_total = v_total + semi_inf(panel.right_tip, wind_dir, -gamma);
    
    return v_total;
}

void VLMModel::solve_linear_system(Vector<Real>& A, Vector<Real>& b, Vector<Real>& x) {
    const int n = static_cast<int>(b.size());
    x.assign(n, 0.0);

    // 1. Gaussian Elimination with Partial Pivoting
    for (int i = 0; i < n; i++) {
        // Find pivot
        int pivot = i;
        for (int j = i + 1; j < n; j++) {
            if (std::abs(A[j * n + i]) > std::abs(A[pivot * n + i])) {
                pivot = j;
            }
        }

        // Swap rows in A and b
        for (int j = i; j < n; j++) {
            std::swap(A[i * n + j], A[pivot * n + j]);
        }
        std::swap(b[i], b[pivot]);

        // Eliminate
        if (std::abs(A[i * n + i]) < 1e-15) continue; // Singular

        for (int j = i + 1; j < n; j++) {
            Real factor = A[j * n + i] / A[i * n + i];
            b[j] -= factor * b[i];
            for (int k = i; k < n; k++) {
                A[j * n + k] -= factor * A[i * n + k];
            }
        }
    }

    // 2. Back Substitution
    for (int i = n - 1; i >= 0; i--) {
        if (std::abs(A[i * n + i]) < 1e-15) {
            x[i] = 0.0;
            continue;
        }
        Real sum = 0.0;
        for (int j = i + 1; j < n; j++) {
            sum += A[i * n + j] * x[j];
        }
        x[i] = (b[i] - sum) / A[i * n + i];
    }
}

} // namespace aero