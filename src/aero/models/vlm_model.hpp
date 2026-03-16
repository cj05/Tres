#pragma once
#include <vector>
#include "../core/aero_math.hpp" // Ensure this defines Real and Vector3

namespace aero {

// Standard VLM Panel data structure
struct VLMPanel {
    Vector3 left_tip;
    Vector3 right_tip;
    Vector3 collocation_point;
    Vector3 normal;
    Real area = 0.0;
    Real chord = 0.0;
    Real circulation = 0.0;
};

// Template-friendly Vector alias if you aren't using std::vector directly
template <typename T>
using Vector = std::vector<T>;

class VLMModel {
public:
    // The main entry point called by AeroSurface.cpp
    static void solve(Vector<VLMPanel>& panels, const Vector3& wind_velocity, bool debug = false);

    static void build_influence_matrix(const Vector<VLMPanel>& panels, const Vector3& wind_velocity, Vector<Real>& A, Vector<Real>& b);
    static void solve_linear_system(Vector<Real>& A, Vector<Real>& b, Vector<Real>& x);
    
    static Vector3 calculate_horseshoe_velocity(const Vector3& p, const VLMPanel& panel, Real gamma, const Vector3& wind_dir);
    static Vector3 calculate_induced_velocity(const Vector3& p, const Vector3& v1, const Vector3& v2, Real gamma, Real eps2 = 1e-12);
};

} // namespace aero