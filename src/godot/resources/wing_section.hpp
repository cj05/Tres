#pragma once

#include <godot_cpp/classes/resource.hpp>
#include "airfoil_resource.hpp"

namespace godot {

class WingSection : public Resource {
    GDCLASS(WingSection, Resource);

protected:
    static void _bind_methods();

    double span_offset = 0.0;
    double sweep_offset = 0.0;
    double dihedral_angle = 0.0;
    double chord = 1.0;
    double twist = 0.0;
    Ref<AirfoilResource> airfoil_data;

public:
    WingSection();
    ~WingSection();

    void set_span_offset(double p_offset);
    double get_span_offset() const;

    void set_sweep_offset(double p_offset);
    double get_sweep_offset() const;

    void set_dihedral_angle(double p_angle);
    double get_dihedral_angle() const;

    void set_chord(double p_chord);
    double get_chord() const;

    void set_twist(double p_twist);
    double get_twist() const;

    void set_airfoil_data(const Ref<AirfoilResource> &p_data);
    Ref<AirfoilResource> get_airfoil_data() const;
};

} // namespace godot
