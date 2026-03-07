#pragma once

#include "aero/flatplate_model.hpp"
#include <godot_cpp/classes/node3d.hpp>

namespace godot {

class AeroBody : public Node3D {
    GDCLASS(AeroBody, Node3D);

protected:
    static void _bind_methods();

public:
    float area = 10.0f;
    float chord = 2.0f;

    AeroBody();

    void _physics_process(double delta);
};

}
