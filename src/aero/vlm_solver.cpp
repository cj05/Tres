#include "vlm_solver.hpp"
#include <godot_cpp/variant/utility_functions.hpp>

using namespace godot;

void VLMSolver::solve(Vector<VLMPanel> &p_panels, const Vector3 &p_wind_velocity) {
    int N = p_panels.size();
    if (N == 0) return;

    Vector3 wind_dir = p_wind_velocity.normalized();
    if (p_wind_velocity.length() < 0.001) return;

    // 1. Build Influence Matrix A and RHS vector b
    Vector<Vector<double>> A;
    Vector<double> b;
    A.resize(N);
    b.resize(N);

    for (int i = 0; i < N; i++) {
        A.write[i].resize(N);
        b.write[i] = -p_wind_velocity.dot(p_panels[i].normal);

        for (int j = 0; j < N; j++) {
            Vector3 v_unit = _calculate_horseshoe_velocity(p_panels[i].collocation_point, p_panels[j], 1.0, wind_dir);
            A.write[i].write[j] = v_unit.dot(p_panels[i].normal);
        }
    }

    // 2. Solve Ax = b using Gaussian Elimination
    for (int i = 0; i < N; i++) {
        int pivot = i;
        for (int j = i + 1; j < N; j++) {
            if (Math::abs(A[j][i]) > Math::abs(A[pivot][i])) pivot = j;
        }
        Vector<double> temp_row = A[i];
        A.write[i] = A[pivot];
        A.write[pivot] = temp_row;
        double temp_b = b[i];
        b.write[i] = b[pivot];
        b.write[pivot] = temp_b;

        if (Math::abs(A[i][i]) < 1e-15) continue;

        for (int j = i + 1; j < N; j++) {
            double factor = A[j][i] / A[i][i];
            b.write[j] -= factor * b[i];
            for (int k = i; k < N; k++) {
                A.write[j].write[k] -= factor * A[i][k];
            }
        }
    }

    Vector<double> x;
    x.resize(N);
    for (int i = N - 1; i >= 0; i--) {
        double sum = 0;
        for (int j = i + 1; j < N; j++) {
            sum += A[i][j] * x[j];
        }
        if (Math::abs(A[i][i]) > 1e-15) {
            x.write[i] = (b[i] - sum) / A[i][i];
        } else {
            x.write[i] = 0;
        }
    }

    for (int i = 0; i < N; i++) {
        p_panels.write[i].circulation = x[i];
    }
}

Vector3 VLMSolver::_calculate_induced_velocity(const Vector3 &p_point, const Vector3 &p_v1, const Vector3 &p_v2, double p_gamma) {
    Vector3 r1 = p_point - p_v1;
    Vector3 r2 = p_point - p_v2;
    Vector3 r0 = r1 - r2;
    Vector3 r1xr2 = r1.cross(r2);
    double r1xr2_mag2 = r1xr2.length_squared();

    const double epsilon2 = 1e-8;
    if (r1xr2_mag2 < epsilon2) return Vector3();

    double r1_mag = r1.length();
    double r2_mag = r2.length();
    if (r1_mag < 1e-6 || r2_mag < 1e-6) return Vector3();

    double term = (p_gamma / (4.0 * Math_PI)) * (r0.dot(r1 / r1_mag - r2 / r2_mag) / (r1xr2_mag2 + epsilon2));
    return r1xr2 * term;
}

Vector3 VLMSolver::_calculate_horseshoe_velocity(const Vector3 &p_point, const VLMPanel &p_panel, double p_gamma, const Vector3 &p_wind_dir) {
    Vector3 v_induced;
    v_induced += _calculate_induced_velocity(p_point, p_panel.left_tip, p_panel.right_tip, p_gamma);

    const double DOWNSTREAM_DIST = 1000.0;
    Vector3 trailing_left_end = p_panel.left_tip + p_wind_dir * DOWNSTREAM_DIST;
    Vector3 trailing_right_end = p_panel.right_tip + p_wind_dir * DOWNSTREAM_DIST;

    v_induced += _calculate_induced_velocity(p_point, trailing_left_end, p_panel.left_tip, p_gamma);
    v_induced += _calculate_induced_velocity(p_point, p_panel.right_tip, trailing_right_end, p_gamma);

    return v_induced;
}
