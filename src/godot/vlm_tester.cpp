#include "vlm_tester.hpp"
#include "aero/vlm_solver.hpp"
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/utility_functions.hpp>
#include <godot_cpp/classes/engine.hpp>

using namespace godot;

VLMTester::VLMTester() {}
VLMTester::~VLMTester() {}

void VLMTester::_ready() {
    if (!Engine::get_singleton()->is_editor_hint()) {
        run_vlm_validation();
    }
}

void VLMTester::_bind_methods() {
    ClassDB::bind_method(D_METHOD("run_vlm_validation"), &VLMTester::run_vlm_validation);
}

void VLMTester::run_vlm_validation() {
    UtilityFunctions::print("--- Starting VLM Validation Test ---");

    // Test Case: Rectangular Wing
    // Span = 10m, Chord = 1m
    // 10 panels along span
    const int num_panels = 10;
    const double span = 10.0;
    const double chord = 1.0;
    const double alpha_deg = 5.0;
    const double alpha_rad = Math::deg_to_rad(alpha_deg);
    const Vector3 wind_velocity(20.0 * Math::cos(alpha_rad), 20.0 * Math::sin(alpha_rad), 0.0); // 20m/s in XY plane

    Vector<VLMPanel> panels;
    for (int i = 0; i < num_panels; i++) {
        VLMPanel p;
        double y_start = -span / 2.0 + (span / num_panels) * i;
        double y_end = y_start + (span / num_panels);
        double y_mid = (y_start + y_end) / 2.0;

        // Place bound vortex at 1/4 chord
        p.left_tip = Vector3(0.25 * chord, 0, y_start);
        p.right_tip = Vector3(0.25 * chord, 0, y_end);
        
        // Collocation point at 3/4 chord
        p.collocation_point = Vector3(0.75 * chord, 0, y_mid);
        p.normal = Vector3(0, 1, 0); // Flat wing
        p.area = (span / num_panels) * chord;
        p.chord = chord;
        
        panels.push_back(p);
    }

    VLMSolver::solve(panels, wind_velocity);

    // Verify Results
    // 1. Check for symmetry
    bool symmetrical = true;
    for (int i = 0; i < num_panels / 2; i++) {
        double g1 = panels[i].circulation;
        double g2 = panels[num_panels - 1 - i].circulation;
        if (Math::abs(g1 - g2) > 1e-10) {
            symmetrical = false;
            UtilityFunctions::print("FAIL: Asymmetry detected between panel ", i, " and ", num_panels-1-i);
        }
    }
    if (symmetrical) UtilityFunctions::print("PASS: Symmetry check passed.");

    // 2. Check lift distribution (should be highest at center)
    bool lift_trend_ok = true;
    for (int i = 0; i < num_panels / 2 - 1; i++) {
        if (panels[i+1].circulation < panels[i].circulation - 1e-10) {
            lift_trend_ok = false;
            UtilityFunctions::print("FAIL: Lift should increase towards center. Panel ", i+1, " gamma < Panel ", i);
        }
    }
    if (lift_trend_ok) UtilityFunctions::print("PASS: Lift distribution trend check passed.");

    // 3. Overall Lift Coefficient check (Approximate)
    // CL = sum(L) / (0.5 * rho * V^2 * S)
    double total_gamma_span = 0;
    for (int i = 0; i < num_panels; i++) {
        total_gamma_span += panels[i].circulation * (span / num_panels);
    }
    
    double rho = 1.225;
    double V = wind_velocity.length();
    double total_lift = rho * V * total_gamma_span;
    double CL = total_lift / (0.5 * rho * V * V * span * chord);
    
    // Theoretical 2D Cl = 2 * PI * alpha
    // Theoretical 3D CL ~ (2*PI*alpha) / (1 + (2*PI*alpha)/(PI*AR)) where AR = span/chord
    double AR = span / chord;
    double CL_theory = (2.0 * Math_PI * alpha_rad) / (1.0 + 2.0 / AR);
    
    UtilityFunctions::print("Calculated CL: ", CL);
    UtilityFunctions::print("Theoretical CL (approx): ", CL_theory);

    if (Math::abs(CL - CL_theory) < 0.1) {
        UtilityFunctions::print("PASS: CL value within reasonable range of theory.");
    } else {
        UtilityFunctions::print("WARN: CL value differs significantly from simple theory.");
    }

    UtilityFunctions::print("--- VLM Validation Test Finished ---");
}
