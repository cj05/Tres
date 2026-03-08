#pragma once

#include "modular_component.hpp"

namespace godot {

class AeroComponent : public ModularComponent {
    GDCLASS(AeroComponent, ModularComponent);

protected:
    static void _bind_methods();

public:
    AeroComponent();
    ~AeroComponent();
};

} // namespace godot
