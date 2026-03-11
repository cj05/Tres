#pragma once

#include "../core/geometry_component.hpp"
#include "../resources/aero_geometry_properties.hpp"
#include <godot_cpp/classes/surface_tool.hpp>

namespace godot {

class AeroGeometry : public GeometryComponent {
    GDCLASS(AeroGeometry, GeometryComponent);

protected:
    static void _bind_methods();

    static const int POINTS_PER_RING = 64;

    PackedVector2Array get_resampled_points(const Ref<AirfoilResource> &airfoil) const;

public:
    AeroGeometry();
    ~AeroGeometry();

    void _generate_geometry() override;
};

} // namespace godot
