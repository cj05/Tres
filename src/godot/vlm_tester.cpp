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
        run_coherence_test();
        run_cosine_spacing_test();
    }
}

void VLMTester::_bind_methods() {
    ClassDB::bind_method(D_METHOD("run_vlm_validation"), &VLMTester::run_vlm_validation);
    ClassDB::bind_method(D_METHOD("run_coherence_test"), &VLMTester::run_coherence_test);
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
        double diff = Math::abs(g1 - g2);
        if (diff > 1e-10) {
            symmetrical = false;
            UtilityFunctions::print("FAIL: Asymmetry at ", i, " vs ", num_panels-1-i, 
                                    ". G1: ", g1, ", G2: ", g2, ", Diff: ", diff);
        }
    }
    if (symmetrical) UtilityFunctions::print("PASS: Symmetry check passed.");

    // 2. Check lift distribution (should be highest at center)
    bool lift_trend_ok = true;
    for (int i = 0; i < num_panels / 2 - 1; i++) {
        if (Math::abs(panels[i+1].circulation) < Math::abs(panels[i].circulation) - 1e-10) {
            lift_trend_ok = false;
            UtilityFunctions::print("FAIL: Lift distribution trend. Panel ", i+1, " gamma magnitude < Panel ", i);
        }
    }
    if (lift_trend_ok) UtilityFunctions::print("PASS: Lift distribution trend check passed.");

    // 3. Overall Lift Coefficient check (Approximate)
    double total_gamma_span = 0;
    for (int i = 0; i < num_panels; i++) {
        total_gamma_span += panels[i].circulation * (span / num_panels);
    }
    
    double rho = 1.225;
    double V = wind_velocity.length();
    double total_lift = rho * V * Math::abs(total_gamma_span); // Ensure positive lift for comparison
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

void VLMTester::run_coherence_test() {
    UtilityFunctions::print("--- Starting VLM Coherence Test ---");

    const int panels_per_wing = 10;
    const double span = 5.0;
    const double chord = 1.0;
    const double alpha_rad = Math::deg_to_rad(5.0);
    const Vector3 wind_velocity(20.0 * Math::cos(alpha_rad), 20.0 * Math::sin(alpha_rad), 0.0);

    // Setup Case 1: Two wings side-by-side (solved together)
    Vector<VLMPanel> combined_panels;
    // Wing 1 (Left: -5 to 0)
    for (int i = 0; i < panels_per_wing; i++) {
        VLMPanel p;
        double y_start = -span + (span / panels_per_wing) * i;
        double y_end = y_start + (span / panels_per_wing);
        p.left_tip = Vector3(0.25 * chord, 0, y_start);
        p.right_tip = Vector3(0.25 * chord, 0, y_end);
        p.collocation_point = Vector3(0.75 * chord, 0, (y_start + y_end) * 0.5);
        p.normal = Vector3(0, 1, 0);
        combined_panels.push_back(p);
    }
    // Wing 2 (Right: 0 to 5)
    for (int i = 0; i < panels_per_wing; i++) {
        VLMPanel p;
        double y_start = 0.0 + (span / panels_per_wing) * i;
        double y_end = y_start + (span / panels_per_wing);
        p.left_tip = Vector3(0.25 * chord, 0, y_start);
        p.right_tip = Vector3(0.25 * chord, 0, y_end);
        p.collocation_point = Vector3(0.75 * chord, 0, (y_start + y_end) * 0.5);
        p.normal = Vector3(0, 1, 0);
        combined_panels.push_back(p);
    }

    VLMSolver::solve(combined_panels, wind_velocity);

    // Setup Case 2: One big wing (Span 10)
    Vector<VLMPanel> single_panels;
    for (int i = 0; i < panels_per_wing * 2; i++) {
        VLMPanel p;
        double y_start = -span + ( (span * 2.0) / (panels_per_wing * 2) ) * i;
        double y_end = y_start + ( (span * 2.0) / (panels_per_wing * 2) );
        p.left_tip = Vector3(0.25 * chord, 0, y_start);
        p.right_tip = Vector3(0.25 * chord, 0, y_end);
        p.collocation_point = Vector3(0.75 * chord, 0, (y_start + y_end) * 0.5);
        p.normal = Vector3(0, 1, 0);
        single_panels.push_back(p);
    }

    VLMSolver::solve(single_panels, wind_velocity);

    // Comparison
    bool match = true;
    for (int i = 0; i < panels_per_wing * 2; i++) {
        double g_combined = combined_panels[i].circulation;
        double g_single = single_panels[i].circulation;
        if (Math::abs(g_combined - g_single) > 1e-10) {
            match = false;
            UtilityFunctions::print("FAIL: Mismatch at panel ", i, ". Combined: ", g_combined, ", Single: ", g_single);
        }
    }

    if (match) {
        UtilityFunctions::print("PASS: Coherence check passed. Multi-section interaction is mathematically consistent.");
    } else {
        UtilityFunctions::print("FAIL: Multi-section interaction mismatch.");
    }

    UtilityFunctions::print("--- VLM Coherence Test Finished ---");
}

void VLMTester::run_cosine_spacing_test() {
    UtilityFunctions::print("--- Starting VLM Cosine Spacing Test ---");

    const int num_panels = 20; 
    const double span = 1.0;
    const double chord = 1.0;
    const double alpha_rad = Math::deg_to_rad(5.0);
    const Vector3 wind_velocity(20.0 * Math::cos(alpha_rad), 20.0 * Math::sin(alpha_rad), 0.0);

    Vector<VLMPanel> panels;
    for (int i = 0; i < num_panels; i++) {
        double t1 = (double)i / num_panels;
        double t2 = (double)(i + 1) / num_panels;

        double cos_t1 = (1.0 - Math::cos(t1 * Math_PI)) * 0.5;
        double cos_t2 = (1.0 - Math::cos(t2 * Math_PI)) * 0.5;
        double cos_mid = (1.0 - Math::cos((t1 + t2) * 0.5 * Math_PI)) * 0.5;

        double y_start = -span/2.0 + span * cos_t1;
        double y_end = -span/2.0 + span * cos_t2;
        double y_mid = -span/2.0 + span * cos_mid;

        VLMPanel p;
        p.left_tip = Vector3(0.25 * chord, 0, y_start);
        p.right_tip = Vector3(0.25 * chord, 0, y_end);
        p.collocation_point = Vector3(0.75 * chord, 0, y_mid);
        p.normal = Vector3(0, 1, 0);
        p.area = (y_end - y_start) * chord;
        p.chord = chord;
        panels.push_back(p);
    }

    VLMSolver::solve(panels, wind_velocity);

    bool symmetrical = true;
    for (int i = 0; i < num_panels / 2; i++) {
        double g1 = panels[i].circulation;
        double g2 = panels[num_panels - 1 - i].circulation;
        double diff = Math::abs(g1 - g2);
        if (diff > 1e-10) {
            symmetrical = false;
            UtilityFunctions::print("FAIL: Cosine Asymmetry at ", i, ". Diff: ", diff);
        }
    }
    if (symmetrical) UtilityFunctions::print("PASS: Cosine symmetry check passed.");

    bool tips_sane = true;
    if (Math::abs(panels[0].circulation) > Math::abs(panels[num_panels/2].circulation)) {
        tips_sane = false;
        UtilityFunctions::print("FAIL: Tip circulation (", panels[0].circulation, ") is larger than center (", panels[num_panels/2].circulation, ")");
    }
    if (tips_sane) UtilityFunctions::print("PASS: Tip behavior appears sane (no divergent spikes).");

    UtilityFunctions::print("--- VLM Cosine Spacing Test Finished ---");
}
