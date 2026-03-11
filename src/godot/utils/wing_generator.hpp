#pragma once

#include <godot_cpp/classes/ref.hpp>
#include <godot_cpp/variant/transform3d.hpp>
#include <godot_cpp/variant/typed_array.hpp>
#include "../resources/aero_geometry_properties.hpp"
#include "../resources/wing_section.hpp"

namespace godot {

struct WingStation {
    Transform3D transform;
    double chord;
    Ref<AirfoilResource> airfoil;
    double twist;
};

struct WingSubsection {
    Transform3D transform;
    double area;
    double chord;
    Ref<AirfoilResource> airfoil;
    double span;
};

class WingGenerator {
public:
    static Vector<WingStation> generate_stations(const Ref<AeroGeometryProperties>& props);
    static Vector<WingSubsection> generate_subsections(const Vector<WingStation>& stations, double segments_per_meter);
};

} // namespace godot
