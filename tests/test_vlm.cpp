#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "aero/models/vlm_model.hpp"

using namespace aero;

TEST_CASE("VLM Component: Induced Velocity (Biot-Savart)") {
    SUBCASE("Zero velocity on the line of the segment") {
        Vector3 v1(0, 0, -1);
        Vector3 v2(0, 0, 1);
        Vector3 p(0, 0, 0); // Directly on the line
        Vector3 vel = VLMModel::calculate_induced_velocity(p, v1, v2, 1.0);
        CHECK(vel.length() == doctest::Approx(0.0).epsilon(1e-12));
    }

    SUBCASE("Symmetry of induced velocity") {
        Vector3 v1(0, 0, -1);
        Vector3 v2(0, 0, 1);
        Vector3 p1(1, 0, 0);
        Vector3 p2(-1, 0, 0);
        Vector3 vel1 = VLMModel::calculate_induced_velocity(p1, v1, v2, 1.0);
        Vector3 vel2 = VLMModel::calculate_induced_velocity(p2, v1, v2, 1.0);
        
        // At p1, velocity should be in Y direction (cross product of Z and X)
        CHECK(std::abs(vel1.y) > 0.0);
        CHECK(vel1.x == doctest::Approx(0.0));
        CHECK(vel1.z == doctest::Approx(0.0));
        
        // Symmetry: vel1 should be -vel2 (opposite directions)
        CHECK(vel1.y == doctest::Approx(-vel2.y));
    }
}

TEST_CASE("VLM Component: Horseshoe Vortex") {
    SUBCASE("Symmetry across centerline") {
        VLMPanel panel;
        panel.left_tip = Vector3(0, 0, -1);
        panel.right_tip = Vector3(0, 0, 1);
        Vector3 wind_dir(1, 0, 0);
        
        Vector3 p_left(0.5, 0, -0.5);
        Vector3 p_right(0.5, 0, 0.5);
        
        Vector3 vel_l = VLMModel::calculate_horseshoe_velocity(p_left, panel, 1.0, wind_dir);
        Vector3 vel_r = VLMModel::calculate_horseshoe_velocity(p_right, panel, 1.0, wind_dir);
        
        // Z-symmetry: Y components should be equal
        CHECK(vel_l.y == doctest::Approx(vel_r.y).epsilon(1e-7));
    }
}

TEST_CASE("VLM Component: Matrix Assembly") {
    const int num_panels = 2;
    Vector<VLMPanel> panels(num_panels);
    panels[0].left_tip = Vector3(0, 0, -1);
    panels[0].right_tip = Vector3(0, 0, 0);
    panels[0].collocation_point = Vector3(0.5, 0, -0.5);
    panels[0].normal = Vector3(0, 1, 0);

    panels[1].left_tip = Vector3(0, 0, 0);
    panels[1].right_tip = Vector3(0, 0, 1);
    panels[1].collocation_point = Vector3(0.5, 0, 0.5);
    panels[1].normal = Vector3(0, 1, 0);

    Vector3 wind_velocity(20, 0, 0);
    Vector<Real> A, b;
    A.resize(num_panels * num_panels);
    b.resize(num_panels);

    VLMModel::build_influence_matrix(panels, wind_velocity, A, b);

    // Diagonal elements should be equal due to symmetry
    CHECK(A[0 * num_panels + 0] == doctest::Approx(A[1 * num_panels + 1]));
    // Off-diagonal elements should be equal due to symmetry
    CHECK(A[0 * num_panels + 1] == doctest::Approx(A[1 * num_panels + 0]));
    
    // RHS vector b should be -V . n
    // V=(20,0,0), n=(0,1,0) -> dot is 0
    CHECK(b[0] == doctest::Approx(0.0));
}

TEST_CASE("VLM Component: Linear Solver") {
    // Solve 2x2: 
    // 2x + 1y = 5
    // 1x + 2y = 4
    // Solution: x=2, y=1
    Vector<Real> A = {2, 1, 1, 2};
    Vector<Real> b = {5, 4};
    Vector<Real> x;
    
    VLMModel::solve_linear_system(A, b, x);
    
    REQUIRE(x.size() == 2);
    CHECK(x[0] == doctest::Approx(2.0));
    CHECK(x[1] == doctest::Approx(1.0));
}

TEST_CASE("VLM Integration: Symmetry and Trends") {
    const int num_panels = 10;
    const Real span = 10.0;
    const Real chord = 1.0;
    const Real alpha_rad = 5.0 * M_PI / 180.0;
    const Vector3 wind_velocity(20.0 * std::cos(alpha_rad), 20.0 * std::sin(alpha_rad), 0.0);

    Vector<VLMPanel> panels;
    for (int i = 0; i < num_panels; i++) {
        VLMPanel p;
        Real y_start = -span / 2.0 + (span / num_panels) * i;
        Real y_end = y_start + (span / num_panels);
        p.left_tip = Vector3(0.25 * chord, 0, y_start);
        p.right_tip = Vector3(0.25 * chord, 0, y_end);
        p.collocation_point = Vector3(0.75 * chord, 0, (y_start + y_end) * 0.5);
        p.normal = Vector3(0, 1, 0);
        p.area = (span / num_panels) * chord;
        p.chord = chord;
        panels.push_back(p);
    }

    VLMModel::solve(panels, wind_velocity);

    for (int i = 0; i < num_panels / 2; i++) {
        Real g1 = panels[i].circulation;
        Real g2 = panels[num_panels - 1 - i].circulation;
        CHECK(std::abs(g1 - g2) == doctest::Approx(0.0).epsilon(1e-7));
    }

    for (int i = 0; i < num_panels / 2 - 1; i++) {
        CHECK(std::abs(panels[i+1].circulation) >= std::abs(panels[i].circulation) - 1e-10);
    }
}

TEST_CASE("VLM Numerical Symmetry: Left vs Right Wing") {
    const int panels_per_side = 10;
    const int total_panels = panels_per_side * 2;
    const Real span = 10.0;
    const Real chord = 1.0;
    const Real alpha_rad = 5.0 * M_PI / 180.0;
    const Vector3 wind_velocity(20.0 * std::cos(alpha_rad), 20.0 * std::sin(alpha_rad), 0.0);

    Vector<VLMPanel> panels;
    // Generate panels from -5 to +5 (full wing)
    for (int i = 0; i < total_panels; i++) {
        VLMPanel p;
        Real y_start = -span / 2.0 + (span / total_panels) * i;
        Real y_end = y_start + (span / total_panels);
        p.left_tip = Vector3(0.25 * chord, 0, y_start);
        p.right_tip = Vector3(0.25 * chord, 0, y_end);
        p.collocation_point = Vector3(0.75 * chord, 0, (y_start + y_end) * 0.5);
        p.normal = Vector3(0, 1, 0);
        p.area = (span / total_panels) * chord;
        p.chord = chord;
        panels.push_back(p);
    }

    VLMModel::solve(panels, wind_velocity);

    // Strict check: circulation at panel i must equal circulation at panel (N-1-i)
    for (int i = 0; i < panels_per_side; i++) {
        Real g_left = panels[i].circulation;
        Real g_right = panels[total_panels - 1 - i].circulation;
        
        MESSAGE("Panel ", i, " (Left) Gamma: ", g_left);
        MESSAGE("Panel ", total_panels - 1 - i, " (Right) Gamma: ", g_right);
        
        // We expect identical results for a symmetric wing in symmetric flow
        CHECK(g_left == doctest::Approx(g_right).epsilon(1e-12));
    }
}

TEST_CASE("VLM Performance: AoA Sweep and Linearity") {
    const int num_panels = 20;
    const Real span = 10.0;
    const Real chord = 1.0;
    const Real rho = 1.225;
    const Real AR = span / chord; // Aspect Ratio = 10

    auto get_total_lift = [&](Real aoa_deg) {
        Real alpha_rad = aoa_deg * M_PI / 180.0;
        Vector3 wind_velocity(20.0 * std::cos(alpha_rad), 20.0 * std::sin(alpha_rad), 0.0);
        
        Vector<VLMPanel> panels;
        for (int i = 0; i < num_panels; i++) {
            VLMPanel p;
            Real y_start = -span / 2.0 + (span / num_panels) * i;
            Real y_end = y_start + (span / num_panels);
            p.left_tip = Vector3(0.25 * chord, 0, y_start);
            p.right_tip = Vector3(0.25 * chord, 0, y_end);
            p.collocation_point = Vector3(0.75 * chord, 0, (y_start + y_end) * 0.5);
            p.normal = Vector3(0, 1, 0);
            p.area = (span / num_panels) * chord;
            p.chord = chord;
            panels.push_back(p);
        }

        VLMModel::solve(panels, wind_velocity);

        Real total_lift = 0;
        Real V = wind_velocity.length();
        Real dy = span / num_panels;
        for (const auto& p : panels) {
            total_lift += rho * V * std::abs(p.circulation) * dy;
        }
        return total_lift;
    };

    SUBCASE("Zero lift at zero AoA") {
        Real lift_0 = get_total_lift(0.0);
        CHECK(std::abs(lift_0) == doctest::Approx(0.0).epsilon(1e-10));
    }

    SUBCASE("Linearity check (Slope consistency)") {
        Real lift_5 = get_total_lift(5.0);
        Real lift_10 = get_total_lift(10.0);
        
        // Lift at 10 deg should be roughly double the lift at 5 deg
        // Using a small epsilon because sin(alpha) is not perfectly linear, but close for small angles.
        CHECK(lift_10 == doctest::Approx(lift_5 * 2.0).epsilon(0.02));
    }

    SUBCASE("Lift Curve Slope (dCL/da)") {
        Real lift_5 = get_total_lift(5.0);
        Real V = 20.0;
        Real q = 0.5 * rho * V * V;
        Real S = span * chord;
        Real CL_5 = lift_5 / (q * S);
        
        Real dCL_da_rad = CL_5 / (5.0 * M_PI / 180.0);
        
        // Theoretical dCL/da for finite wing: 2*pi / (1 + 2/AR)
        // For AR=10: 2*pi / (1 + 0.2) = 2*pi / 1.2 approx 5.23
        Real theory_slope = (2.0 * M_PI) / (1.0 + 2.0 / AR);
        
        MESSAGE("Calculated dCL/da: ", dCL_da_rad);
        MESSAGE("Theoretical dCL/da: ", theory_slope);
        
        // VLM usually matches this within a few percent for rectangular wings
        CHECK(dCL_da_rad == doctest::Approx(theory_slope).epsilon(0.05));
    }
}
