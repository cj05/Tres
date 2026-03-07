#include "aero_body.hpp"
#include <godot_cpp/core/class_db.hpp>

using namespace godot;

AeroBody::AeroBody() {}

void AeroBody::_bind_methods() {
}

void AeroBody::_physics_process(double delta) {

    AeroInput input;

    input.rho   = 1.225f;
    input.speed = 100.0f;
    input.alpha = 0.1f;
    input.area  = area;
    input.chord = chord;

    AeroOutput out = flatplate_compute(input);

    print_line("Lift: ", out.lift);
    print_line("Drag: ", out.drag);
    print_line("Moment: ", out.moment);
}
