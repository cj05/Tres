#include "vlm_solver.hpp"
#include <godot_cpp/variant/utility_functions.hpp>

using namespace godot;

void VLMSolver::solve(Vector<VLMPanel> &p_panels, const Vector3 &p_wind_velocity) {
    int N = p_panels.size();
    if (N == 0) return;

    Vector3 wind_dir = p_wind_velocity.normalized();
    double wind_speed = p_wind_velocity.length();
    if (wind_speed < 0.001) return;

    Vector<double> A;
    A.resize(N * N);
    Vector<double> b;
    b.resize(N);

    // 1. Build Influence Matrix
    for (int i = 0; i < N; i++) {
        b.write[i] = -p_wind_velocity.dot(p_panels[i].normal);

        for (int j = 0; j < N; j++) {
            Vector3 v_unit = _calculate_horseshoe_velocity(p_panels[i].collocation_point, p_panels[j], 1.0, wind_dir);
            A.write[i * N + j] = v_unit.dot(p_panels[i].normal);
        }
    }

    // 2. Pre-conditioning: Row Scaling
    // This helps significantly with ill-conditioned matrices from cosine spacing
    for (int i = 0; i < N; i++) {
        double diag = Math::abs(A[i * N + i]);
        if (diag > 1e-12) {
            double scale = 1.0 / A[i * N + i];
            for (int j = 0; j < N; j++) {
                A.write[i * N + j] *= scale;
            }
            b.write[i] *= scale;
        }
    }

    // 3. Solve Ax = b with partial pivoting
    for (int i = 0; i < N; i++) {
        int pivot = i;
        double max_val = Math::abs(A[i * N + i]);
        for (int j = i + 1; j < N; j++) {
            double val = Math::abs(A[j * N + i]);
            if (val > max_val) {
                max_val = val;
                pivot = j;
            }
        }
        
        if (pivot != i) {
            for (int k = 0; k < N; k++) {
                double temp = A[i * N + k];
                A.write[i * N + k] = A[pivot * N + k];
                A.write[pivot * N + k] = temp;
            }
            double temp_b = b[i];
            b.write[i] = b[pivot];
            b.write[pivot] = temp_b;
        }

        if (Math::abs(A[i * N + i]) < 1e-18) continue;

        for (int j = i + 1; j < N; j++) {
            double factor = A[j * N + i] / A[i * N + i];
            b.write[j] -= factor * b[i];
            for (int k = i; k < N; k++) {
                A.write[j * N + k] -= factor * A[i * N + k];
            }
        }
    }

    Vector<double> x;
    x.resize(N);
    for (int i = N - 1; i >= 0; i--) {
        double sum = 0;
        for (int j = i + 1; j < N; j++) sum += A[i * N + j] * x[j];
        if (Math::abs(A[i * N + i]) > 1e-18) x.write[i] = (b[i] - sum) / A[i * N + i];
        else x.write[i] = 0;
    }

    for (int i = 0; i < N; i++) p_panels.write[i].circulation = x[i];
}

Vector3 VLMSolver::_calculate_induced_velocity(const Vector3 &p_point, const Vector3 &p_v1, const Vector3 &p_v2, double p_gamma) {
    Vector3 r1 = p_point - p_v1;
    Vector3 r2 = p_point - p_v2;
    Vector3 r1xr2 = r1.cross(r2);
    double r1xr2_mag2 = r1xr2.length_squared();

    Vector3 r0 = p_v2 - p_v1;
    double r0_mag = r0.length();
    
    // Core radius proportional to segment length to prevent singularities
    // This is crucial for tip clusters where panels are tiny
    const double core_radius = MAX(0.05 * r0_mag, 0.0001); 
    const double core_radius2 = core_radius * core_radius;

    double r1_mag = r1.length();
    double r2_mag = r2.length();
    
    if (r1_mag < 1e-10 || r2_mag < 1e-10) return Vector3();

    double term = (p_gamma / (4.0 * Math_PI)) * (r0.dot(r1 / r1_mag - r2 / r2_mag));
    // Softened distance denominator
    double denom = r1xr2_mag2 + core_radius2 * r0_mag * r0_mag;

    return r1xr2 * (term / (denom + 1e-15));
}

Vector3 VLMSolver::_calculate_horseshoe_velocity(const Vector3 &p_point, const VLMPanel &p_panel, double p_gamma, const Vector3 &p_wind_dir) {
    Vector3 v_total;

    // 1. Bound segment
    v_total += _calculate_induced_velocity(p_point, p_panel.left_tip, p_panel.right_tip, p_gamma);

    // 2. Trailing segments (Semi-infinite)
    // Wake core radius proportional to local chord
    const double wake_core = MAX(0.1 * p_panel.chord, 0.001);
    const double wake_core2 = wake_core * wake_core;

    auto calculate_semi_infinite = [&](const Vector3 &P1, const Vector3 &D, double G) {
        Vector3 r1 = p_point - P1;
        Vector3 Dxr1 = D.cross(r1);
        double Dxr1_mag2 = Dxr1.length_squared();
        double r1_mag = r1.length();
        if (r1_mag < 1e-10) return Vector3();

        // Use chord-scaled damping
        return Dxr1 * ( (G / (4.0 * Math_PI)) * (1.0 + D.dot(r1) / r1_mag) / (Dxr1_mag2 + wake_core2) );
    };

    v_total += calculate_semi_infinite(p_panel.left_tip, p_wind_dir, p_gamma);
    v_total += calculate_semi_infinite(p_panel.right_tip, p_wind_dir, p_gamma);

    return v_total;
}
