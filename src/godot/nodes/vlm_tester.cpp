#include "vlm_tester.hpp"
#include "aero_surface.hpp"
#include "aero/models/vlm_model.hpp"
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/utility_functions.hpp>
#include <godot_cpp/classes/engine.hpp>

using namespace godot;

// Conversion helpers
static aero::Vector3 to_aero(const Vector3& v) { return aero::Vector3(v.x, v.y, v.z); }

VLMTester::VLMTester() {}
VLMTester::~VLMTester() {}

void VLMTester::set_aero_surface(Node *p_surface) {
    aero_surface = Object::cast_to<AeroSurface>(p_surface);
}

Node *VLMTester::get_aero_surface() const {
    return aero_surface;
}

void VLMTester::set_run_tests(bool p_run) {
    if (p_run) run_all_tests();
}

void VLMTester::_ready() {
    if (!Engine::get_singleton()->is_editor_hint()) {
        run_all_tests();
    }
}

void VLMTester::run_all_tests() {
    last_result = "Running...\n";
    UtilityFunctions::print("--- Running VLMTester Suite ---");
    run_vlm_validation();
    run_coherence_test();
    run_range_stability_test();
    run_cosine_spacing_test();
    run_aoa_sweep_test();
    UtilityFunctions::print("--- VLMTester Suite Finished ---");
}

void VLMTester::_report(const String& msg, bool success) {
    String line = (success ? "[PASS] " : "[FAIL] ") + msg + "\n";
    last_result += line;
    if (verbose) UtilityFunctions::print(line);
}

void VLMTester::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_aero_surface", "surface"), &VLMTester::set_aero_surface);
    ClassDB::bind_method(D_METHOD("get_aero_surface"), &VLMTester::get_aero_surface);
    ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "aero_surface", PROPERTY_HINT_NODE_TYPE, "AeroSurface"), "set_aero_surface", "get_aero_surface");

    ClassDB::bind_method(D_METHOD("set_run_tests", "run"), &VLMTester::set_run_tests);
    ClassDB::bind_method(D_METHOD("get_run_tests"), &VLMTester::get_run_tests);
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "run_all_tests"), "set_run_tests", "get_run_tests");

    ClassDB::bind_method(D_METHOD("set_verbose", "verbose"), &VLMTester::set_verbose);
    ClassDB::bind_method(D_METHOD("is_verbose"), &VLMTester::is_verbose);
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "verbose"), "set_verbose", "is_verbose");

    ClassDB::bind_method(D_METHOD("set_last_result", "res"), &VLMTester::set_last_result);
    ClassDB::bind_method(D_METHOD("get_last_result"), &VLMTester::get_last_result);
    ADD_PROPERTY(PropertyInfo(Variant::STRING, "last_result", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_EDITOR | PROPERTY_USAGE_READ_ONLY), "set_last_result", "get_last_result");

    ClassDB::bind_method(D_METHOD("run_vlm_validation"), &VLMTester::run_vlm_validation);
    ClassDB::bind_method(D_METHOD("run_coherence_test"), &VLMTester::run_coherence_test);
    ClassDB::bind_method(D_METHOD("run_cosine_spacing_test"), &VLMTester::run_cosine_spacing_test);
    ClassDB::bind_method(D_METHOD("run_aoa_sweep_test"), &VLMTester::run_aoa_sweep_test);
    ClassDB::bind_method(D_METHOD("run_detailed_debug"), &VLMTester::run_detailed_debug);
}

void VLMTester::run_vlm_validation() {
    _report("Starting VLM Validation Test");

    if (aero_surface) {
        // 1. Check for orientation relative to wing normal
        // Use a test wind that gives positive local AoA
        // Headwind from "below" the wing (+Y in local)
        Transform3D tf = aero_surface->get_global_transform();
        Vector3 local_wind(20.0, 1.3, 0.0); // Air from front (+X) and below (+Y)
        aero_surface->set_wind_velocity(tf.basis.xform(local_wind));
        
        (void)aero_surface->compute_force(Variant());
        TypedArray<Dictionary> v_data = aero_surface->get_vortices();
        
        if (v_data.size() > 0) {
            Dictionary v = v_data[0];
            double lift = v["lift"];
            if (lift > 1.0) {
                _report("Lift is correctly oriented relative to wing normal (Local Lift: " + String::num(lift, 2) + ")");
            } else {
                _report("LIFT DIRECTION ERROR! Local Lift: " + String::num(lift, 2) + " (Expected > 0)", false);
            }
        }

        Vector3 force = aero_surface->get_force_cache();
        if (Math::abs(force.z) < 1e-4) {
            _report("Symmetry check passed.");
        } else {
            _report("Asymmetry detected: Z=" + String::num(force.z, 5), false);
        }
    } else {
        _report("No AeroSurface assigned for validation.", false);
    }
}


void VLMTester::run_coherence_test() {
    _report("Starting VLM Coherence Test");
    if (!aero_surface) return;

    (void)aero_surface->compute_force(Variant());
    
    double sum_individual_lift = 0.0;
    TypedArray<Dictionary> v_data = aero_surface->get_vortices();
    for (int i = 0; i < v_data.size(); i++) {
        Dictionary v = v_data[i];
        Vector3 lv = v["lift_vector"];
        sum_individual_lift += lv.length();
    }

    Vector3 total_force = aero_surface->get_force_cache();
    double error = Math::abs(sum_individual_lift - total_force.length());

    if (error < 1e-4) {
        _report("Force summation is coherent (Error: " + String::num(error, 6) + ")");
    } else {
        _report("Force incoherence! Sum: " + String::num(sum_individual_lift) + " Cache: " + String::num(total_force.length()), false);
    }
}

void VLMTester::run_cosine_spacing_test() {
    _report("Starting VLM Cosine Spacing Test");

    const int num_panels = 20; 
    const double span = 1.0;
    const double chord = 1.0;
    const double alpha_rad = Math::deg_to_rad(5.0);
    const Vector3 wind_velocity(20.0 * Math::cos(alpha_rad), 20.0 * Math::sin(alpha_rad), 0.0);

    aero::Vector<aero::VLMPanel> panels;
    for (int i = 0; i < num_panels; i++) {
        double t1 = (double)i / num_panels;
        double t2 = (double)(i + 1) / num_panels;
        double cos_t1 = (1.0 - Math::cos(t1 * Math_PI)) * 0.5;
        double cos_t2 = (1.0 - Math::cos(t2 * Math_PI)) * 0.5;
        double cos_mid = (1.0 - Math::cos((t1 + t2) * 0.5 * Math_PI)) * 0.5;

        aero::VLMPanel p;
        p.left_tip = to_aero(Vector3(0.25 * chord, 0, -span/2.0 + span * cos_t1));
        p.right_tip = to_aero(Vector3(0.25 * chord, 0, -span/2.0 + span * cos_t2));
        p.collocation_point = to_aero(Vector3(0.75 * chord, 0, -span/2.0 + span * cos_mid));
        p.normal = to_aero(Vector3(0, 1, 0));
        p.area = (cos_t2 - cos_t1) * span * chord;
        p.chord = chord;
        panels.push_back(p);
    }

    aero::VLMModel::solve(panels, to_aero(wind_velocity));

    bool symmetrical = true;
    for (int i = 0; i < num_panels / 2; i++) {
        double diff = Math::abs(panels[i].circulation - panels[num_panels - 1 - i].circulation);
        if (diff > 1e-7) symmetrical = false;
    }
    
    if (symmetrical) {
        _report("Cosine symmetry check passed.");
    } else {
        _report("Cosine symmetry failure!", false);
    }

    if (Math::abs(panels[0].circulation) < Math::abs(panels[num_panels/2].circulation)) {
        _report("Tip circulation is correctly lower than center.");
    } else {
        _report("Tip circulation spike detected!", false);
    }
}


void VLMTester::run_range_stability_test() {
    _report("Starting VLM Range & Stability Test");

    if (!aero_surface) return;

    const double speed = 20.0;
    const double rho = 1.225;
    const double dyn_press = 0.5 * rho * speed * speed;
    double test_aoas[] = { 0.0, 5.0, 15.0, 25.0 };
    
    for (double aoa : test_aoas) {
        double rad = Math::deg_to_rad(aoa);
        Vector3 wind(speed * Math::cos(rad), speed * Math::sin(rad), 0.0);
        aero_surface->set_wind_velocity(wind);
        (void)aero_surface->compute_force(Variant());

        TypedArray<Dictionary> v_data = aero_surface->get_vortices();
        bool failed = false;
        for (int i = 0; i < v_data.size(); i++) {
            Dictionary v = v_data[i];
            double gamma = v["circulation"];
            if (!Math::is_finite(gamma) || Math::abs(gamma) > 1000.0) {
                failed = true;
                break;
            }
        }
        
        if (!failed) {
            _report("AoA " + String::num(aoa) + " deg is stable.");
        } else {
            _report("AoA " + String::num(aoa) + " deg diverged!", false);
        }
    }
}

void VLMTester::run_aoa_sweep_test() {
    _report("Starting VLM AoA Sweep Test");
    if (!aero_surface) return;

    for (float aoa_deg = -5.0; aoa_deg <= 20.1; aoa_deg += 5.0) {
        double alpha_rad = Math::deg_to_rad(aoa_deg);
        Vector3 wind(20.0 * Math::cos(alpha_rad), 20.0 * Math::sin(alpha_rad), 0.0);
        aero_surface->set_wind_velocity(wind);
        Vector3 force = aero_surface->compute_force(Variant());
        _report("AoA: " + String::num(aoa_deg) + " Lift: " + String::num(force.y, 1));
    }
}

void VLMTester::run_detailed_debug() {
    _report("Starting VLM Detailed Debug");
    const int num_panels = 3;
    const double span = 10.0;
    const double chord = 1.0;
    const double alpha_deg = 5.0;
    const double alpha_rad = Math::deg_to_rad(alpha_deg);
    const Vector3 wind_velocity(20.0 * Math::cos(alpha_rad), 20.0 * Math::sin(alpha_rad), 0.0);

    aero::Vector<aero::VLMPanel> panels;
    double dy = span / num_panels;
    for (int i = 0; i < num_panels; i++) {
        aero::VLMPanel p;
        double y_start = -span / 2.0 + dy * i;
        double y_end = y_start + dy;
        // Standard wing along Z, chord along X
        p.left_tip = to_aero(Vector3(0.25 * chord, 0, y_start));
        p.right_tip = to_aero(Vector3(0.25 * chord, 0, y_end));
        p.collocation_point = to_aero(Vector3(0.75 * chord, 0, (y_start + y_end) * 0.5));
        p.normal = to_aero(Vector3(0, 1, 0));
        p.area = dy * chord;
        p.chord = chord;
        panels.push_back(p);
    }

    // Solve with debug flag = true
    aero::VLMModel::solve(panels, to_aero(wind_velocity), true);

    for (int i = 0; i < num_panels; i++) {
        String msg = "Panel " + String::num(i) + ": Gamma=" + String::num(panels[i].circulation, 4);
        _report(msg);
    }
}

