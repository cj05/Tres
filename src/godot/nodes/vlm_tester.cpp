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

void VLMTester::_ready() {
    if (!Engine::get_singleton()->is_editor_hint()) {
        run_vlm_validation();
        run_coherence_test();
        run_range_stability_test();
        run_cosine_spacing_test();
        run_aoa_sweep_test();
    }
}

void VLMTester::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_aero_surface", "surface"), &VLMTester::set_aero_surface);
    ClassDB::bind_method(D_METHOD("get_aero_surface"), &VLMTester::get_aero_surface);
    ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "aero_surface", PROPERTY_HINT_NODE_TYPE, "AeroSurface"), "set_aero_surface", "get_aero_surface");

    ClassDB::bind_method(D_METHOD("run_vlm_validation"), &VLMTester::run_vlm_validation);
    ClassDB::bind_method(D_METHOD("run_coherence_test"), &VLMTester::run_coherence_test);
    ClassDB::bind_method(D_METHOD("run_cosine_spacing_test"), &VLMTester::run_cosine_spacing_test);
    ClassDB::bind_method(D_METHOD("run_aoa_sweep_test"), &VLMTester::run_aoa_sweep_test);
}

void VLMTester::run_vlm_validation() {
    UtilityFunctions::print("--- Starting VLM Validation Test ---");

    if (aero_surface) {
        UtilityFunctions::print("Using AeroSurface node: ", aero_surface->get_name());
        
        // 1. Check for inverted lift
        const double alpha_deg = 5.0;
        const double alpha_rad = Math::deg_to_rad(alpha_deg);
        const double speed = 20.0;
        // Wind from front-bottom (positive alpha)
        const Vector3 wind_velocity(speed * Math::cos(alpha_rad), speed * Math::sin(alpha_rad), 0.0);
        
        aero_surface->set_wind_velocity(wind_velocity);
        Vector3 force = aero_surface->compute_force(Variant());
        
        // Lift should be positive in the Y direction if the wing is aligned to axes
        UtilityFunctions::print("AeroSurface Force at 5 deg AoA: ", force);
        if (force.y > 0) {
            UtilityFunctions::print("PASS: Lift is NOT inverted.");
        } else if (force.y < 0) {
            UtilityFunctions::print("FAIL: LIFT IS INVERTED! (Force Y: ", force.y, ")");
        } else {
            UtilityFunctions::print("WARNING: Zero lift detected at 5 deg AoA.");
        }

        // 2. Check for Symmetry if applicable (assuming standard wing)
        if (Math::abs(force.z) < 1e-5) {
            UtilityFunctions::print("PASS: Lateral force is near zero (Symmetry check).");
        } else {
            UtilityFunctions::print("FAIL: Significant lateral force detected: ", force.z);
        }
    } else {
        UtilityFunctions::print("No AeroSurface assigned, running fallback manual test.");
        
        const int num_panels = 10;
        const double span = 10.0;
        const double chord = 1.0;
        const double alpha_rad = Math::deg_to_rad(5.0);
        const Vector3 wind_velocity(20.0 * Math::cos(alpha_rad), 20.0 * Math::sin(alpha_rad), 0.0);

        aero::Vector<aero::VLMPanel> panels;
        for (int i = 0; i < num_panels; i++) {
            aero::VLMPanel p;
            double y_start = -span / 2.0 + (span / num_panels) * i;
            double y_end = y_start + (span / num_panels);
            p.left_tip = to_aero(Vector3(0.25 * chord, 0, y_start));
            p.right_tip = to_aero(Vector3(0.25 * chord, 0, y_end));
            p.collocation_point = to_aero(Vector3(0.75 * chord, 0, (y_start + y_end) * 0.5));
            p.normal = to_aero(Vector3(0, 1, 0));
            p.area = (span / num_panels) * chord;
            p.chord = chord;
            panels.push_back(p);
        }

        aero::VLMModel::solve(panels, to_aero(wind_velocity));

        bool symmetrical = true;
        for (int i = 0; i < num_panels / 2; i++) {
            double g1 = panels[i].circulation;
            double g2 = panels[num_panels - 1 - i].circulation;
            double diff = Math::abs(g1 - g2);
            if (diff > 1e-7) {
                symmetrical = false;
                UtilityFunctions::print("FAIL: Asymmetry at ", i, " vs ", num_panels-1-i, ". Diff: ", diff);
            }
        }
        if (symmetrical) UtilityFunctions::print("PASS: Symmetry check passed.");

        bool lift_trend_ok = true;
        for (int i = 0; i < num_panels / 2 - 1; i++) {
            if (Math::abs(panels[i+1].circulation) < Math::abs(panels[i].circulation) - 1e-10) {
                lift_trend_ok = false;
                UtilityFunctions::print("FAIL: Lift distribution trend failure.");
            }
        }
        if (lift_trend_ok) UtilityFunctions::print("PASS: Lift distribution trend check passed.");

        double total_gamma_span = 0;
        for (int i = 0; i < num_panels; i++) total_gamma_span += panels[i].circulation * (span / num_panels);
        double rho = 1.225;
        double V = wind_velocity.length();
        double CL = (rho * V * Math::abs(total_gamma_span)) / (0.5 * rho * V * V * span * chord);
        UtilityFunctions::print("Calculated CL: ", CL);
    }

    UtilityFunctions::print("--- VLM Validation Test Finished ---");
}


void VLMTester::run_coherence_test() {
    UtilityFunctions::print("--- Starting VLM Coherence Test ---");
    if (!aero_surface) return;

    // 1. Solve the system normally
    aero_surface->compute_force(Variant());
    
    // 2. Cross-reference: Kutta-Joukowski vs. Trefftz Plane (or Momentum)
    // For a simple coherence check: The sum of individual panel lifts 
    // should equal the total force magnitude returned by compute_force.
    
    double sum_individual_lift = 0.0;
    // (Assuming you've added a way to access the internal 'vortices' vector)
    TypedArray<Dictionary> v_data = aero_surface->get_vortices();
    for (int i = 0; i < v_data.size(); i++) {
        Dictionary v = v_data[i]; // Cast Variant to Dictionary
        Vector3 lv = v["lift_vector"]; // Dictionary handles the Variant conversion
        sum_individual_lift += lv.length();
    }

    Vector3 total_force = aero_surface->get_force_cache();
    double error = Math::abs(sum_individual_lift - total_force.length());

    if (error < 1e-4) {
        UtilityFunctions::print("PASS: Force summation is coherent. Error: ", error);
    } else {
        UtilityFunctions::print("FAIL: Force incoherence! Sum: ", sum_individual_lift, " Cache: ", total_force.length());
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

    UtilityFunctions::print("Cosine results (Tips -> Center):");
    String g_vals = "";
    for(int i=0; i<num_panels; i++) g_vals += String::num(panels[i].circulation, 4) + " ";
    UtilityFunctions::print(g_vals);

    bool symmetrical = true;
    for (int i = 0; i < num_panels / 2; i++) {
        double diff = Math::abs(panels[i].circulation - panels[num_panels - 1 - i].circulation);
        if (diff > 1e-7) {
            symmetrical = false;
            UtilityFunctions::print("FAIL: Cosine Asymmetry at ", i, ". Diff: ", diff);
        }
    }
    if (symmetrical) UtilityFunctions::print("PASS: Cosine symmetry check passed.");

    if (Math::abs(panels[0].circulation) < Math::abs(panels[num_panels/2].circulation)) {
        UtilityFunctions::print("PASS: Tip circulation is correctly lower than center.");
    } else {
        UtilityFunctions::print("FAIL: Tip circulation spike detected.");
    }

    UtilityFunctions::print("--- VLM Cosine Spacing Test Finished ---");
}


void VLMTester::run_range_stability_test() {
    UtilityFunctions::print("--- Starting VLM Range & Stability Test ---");

    if (!aero_surface) {
        UtilityFunctions::print("FAIL: No AeroSurface to test.");
        return;
    }

    const double speed = 20.0;
    const double rho = 1.225;
    const double dyn_press = 0.5 * rho * speed * speed;

    // Test across a range of AoAs to find the "Break Point"
    double test_aoas[] = { 0.0, 5.0, 15.0, 25.0 };
    
    for (double aoa : test_aoas) {
        double rad = Math::deg_to_rad(aoa);
        Vector3 wind(speed * Math::cos(rad), speed * Math::sin(rad), 0.0);
        
        aero_surface->set_wind_velocity(wind);
        aero_surface->compute_force(Variant());

        // Check every panel's circulation
        bool panel_failed = false;
        double max_observed_cl = 0.0;

        // You'll need to expose the vortices vector or get results from AeroSurface
        TypedArray<Dictionary> subs = aero_surface->get_subsections();
        TypedArray<Dictionary> v_data_range = aero_surface->get_vortices();

        for (int i = 0; i < v_data_range.size(); i++) {
            Dictionary v = v_data_range[i];
            Dictionary s = subs[i];
            
            double gamma = v["circulation"];
            double area = s["area"];
            //double chord = s["chord"];
            
            // Calculate local CL: Lift / (q * S)
            // Using Kutta-Joukowski Lift approximation: rho * V * gamma * span
            Dictionary sub = aero_surface->get_subsections()[i];
            double chord_val = (double)sub["chord"]; 
            double span = area / chord_val;
            double lift = rho * speed * Math::abs(gamma) * span;
            double cl = (area > 0) ? lift / (dyn_press * area) : 0.0;

            max_observed_cl = Math::max(max_observed_cl, cl);

            // 1. Check for NaN/Inf
            if (!Math::is_finite(gamma)) {
                UtilityFunctions::print("FAIL: NaN/Inf detected at AoA ", aoa, " on panel ", i);
                panel_failed = true;
                break;
            }

            // 2. Check for "Physical Impossible" Range
            // A CL > 10.0 in a standard VLM usually means the 3.232 AR spike is happening
            if (cl > 10.0) {
                UtilityFunctions::print("FAIL: Divergent Lift! Panel ", i, " CL is ", cl, " (Limit: 10.0)");
                panel_failed = true;
                break;
            }
        }

        if (!panel_failed) {
            UtilityFunctions::print("AoA ", aoa, " deg: STABLE (Max CL: ", max_observed_cl, ")");
        }
    }

    UtilityFunctions::print("--- VLM Range & Stability Test Finished ---");
}

void VLMTester::run_aoa_sweep_test() {
    UtilityFunctions::print("--- Starting VLM AoA Sweep Test ---");

    if (aero_surface) {
        const double rho = 1.225;
        const double V = 20.0;
        
        for (float aoa_deg = -10.0; aoa_deg <= 40.1; aoa_deg += 5.0) {
            double alpha_rad = Math::deg_to_rad(aoa_deg);
            Vector3 wind_velocity(V * Math::cos(alpha_rad), V * Math::sin(alpha_rad), 0.0);
            
            aero_surface->set_wind_velocity(wind_velocity);
            Vector3 force = aero_surface->compute_force(Variant());
            
            // Note: CL here is a rough approximation because we don't have total area exposed easily,
            // but we can at least check the trend.
            UtilityFunctions::print("AeroSurface AoA:", aoa_deg, " ForceY:", force.y);
        }
    } else {
        const int num_panels = 10;
        const double span = 10.0;
        const double chord = 1.0;
        const double rho = 1.225;
        const double V = 20.0;

        for (float aoa_deg = -10.0; aoa_deg <= 40.1; aoa_deg += 5.0) {
            double alpha_rad = Math::deg_to_rad(aoa_deg);
            Vector3 wind_velocity(V * Math::cos(alpha_rad), V * Math::sin(alpha_rad), 0.0);

            aero::Vector<aero::VLMPanel> panels;
            double dy = span / num_panels;
            for (int i = 0; i < num_panels; i++) {
                aero::VLMPanel p;
                double y_start = -span/2.0 + dy * i;
                double y_end = y_start + dy;
                p.left_tip = to_aero(Vector3(0.25 * chord, 0, y_start));
                p.right_tip = to_aero(Vector3(0.25 * chord, 0, y_end));
                p.collocation_point = to_aero(Vector3(0.75 * chord, 0, (y_start + y_end) * 0.5));
                p.normal = to_aero(Vector3(0,1,0));
                p.area = dy * chord;
                p.chord = chord;
                panels.push_back(p);
            }

            aero::VLMModel::solve(panels, to_aero(wind_velocity));

            double total_lift = 0;
            for (int i = 0; i < num_panels; i++) {
                total_lift += rho * V * panels[i].circulation * dy;
            }

            double CL = total_lift / (0.5 * rho * V * V * span * chord);
            UtilityFunctions::print("AoA:", aoa_deg, " CL:", CL, " TotalLift:", total_lift);
        }
    }

    UtilityFunctions::print("--- VLM AoA Sweep Test Finished ---");
}

