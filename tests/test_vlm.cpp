#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "aero/models/vlm_model.hpp"

using namespace aero;

TEST_CASE("VLM Symmetry Check") {
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
}

TEST_CASE("VLM Lift Trend Check") {
    const int num_panels = 20;
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

    // Circulation should increase from tip to center for a rectangular wing
    for (int i = 0; i < num_panels / 2 - 1; i++) {
        CHECK(std::abs(panels[i+1].circulation) >= std::abs(panels[i].circulation) - 1e-10);
    }
}
