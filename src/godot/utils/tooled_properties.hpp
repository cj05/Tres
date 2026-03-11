#pragma once

#include <godot_cpp/classes/resource.hpp>

namespace godot {

class TooledProperties : public Resource {
    GDCLASS(TooledProperties, Resource);

protected:
    static void _bind_methods();

    double density = 1.0;
    bool override_mass = false;
    double custom_mass = 1.0;

public:
    TooledProperties();
    ~TooledProperties();

    void set_density(double p_density);
    double get_density() const;

    void set_override_mass(bool p_override);
    bool is_override_mass() const;

    void set_custom_mass(double p_mass);
    double get_custom_mass() const;

    // To mimic _validate_property in C++ if needed, but usually we just handle it in setters
};

} // namespace godot
