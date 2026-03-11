#pragma once

#include "../utils/tooled_properties.hpp"
#include "wing_section.hpp"

namespace godot {

class AeroGeometryProperties : public TooledProperties {
    GDCLASS(AeroGeometryProperties, TooledProperties);

protected:
    static void _bind_methods();

    double strength = 1.0;
    TypedArray<WingSection> sections;
    double root_chord = 1.0;
    Ref<AirfoilResource> root_airfoil;

    void _on_sub_resource_changed();

public:
    AeroGeometryProperties();
    ~AeroGeometryProperties();

    void set_strength(double p_strength);
    double get_strength() const;

    void set_sections(const TypedArray<WingSection> &p_sections);
    TypedArray<WingSection> get_sections() const;

    void set_root_chord(double p_chord);
    double get_root_chord() const;

    void set_root_airfoil(const Ref<AirfoilResource> &p_airfoil);
    Ref<AirfoilResource> get_root_airfoil() const;
};

} // namespace godot
