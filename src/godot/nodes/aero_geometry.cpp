#include "aero_geometry.hpp"
#include "../utils/wing_generator.hpp"
#include <godot_cpp/core/class_db.hpp>

using namespace godot;

AeroGeometry::AeroGeometry() {
}

AeroGeometry::~AeroGeometry() {}

void AeroGeometry::_bind_methods() {
}

void AeroGeometry::_generate_geometry() {
    Ref<AeroGeometryProperties> wing_props = properties;
    if (wing_props.is_null()) return;

    Ref<SurfaceTool> st;
    st.instantiate();
    st->begin(Mesh::PRIMITIVE_TRIANGLES);

    Vector<WingStation> stations = WingGenerator::generate_stations(wing_props);
    Vector<PackedVector3Array> all_stations_points;

    for (const auto& s : stations) {
        PackedVector3Array points_3d;
        PackedVector2Array resampled = get_resampled_points(s.airfoil);
        for (int j = 0; j < resampled.size(); j++) {
            Vector2 p2 = resampled[j];
            Vector3 p3(p2.x * s.chord, p2.y * s.chord, 0);
            p3 = p3.rotated(Vector3(1, 0, 0), Math::deg_to_rad(s.twist));
            points_3d.append(s.transform.xform(p3));
        }
        all_stations_points.append(points_3d);
    }

    // Caps and Sewing
    if (all_stations_points.size() > 0) {
        // Root cap
        PackedVector3Array root_ring = all_stations_points[0];
        for (int i = 1; i < root_ring.size() - 1; i++) {
            st->add_vertex(root_ring[0]);
            st->add_vertex(root_ring[i]);
            st->add_vertex(root_ring[i + 1]);
        }

        // Sewing
        for (int s = 0; s < all_stations_points.size() - 1; s++) {
            PackedVector3Array ring_a = all_stations_points[s];
            PackedVector3Array ring_b = all_stations_points[s + 1];
            int p_count = ring_a.size();

            for (int i = 0; i < p_count; i++) {
                int next_i = (i + 1) % p_count;
                st->add_vertex(ring_a[i]);
                st->add_vertex(ring_b[i]);
                st->add_vertex(ring_a[next_i]);

                st->add_vertex(ring_b[i]);
                st->add_vertex(ring_b[next_i]);
                st->add_vertex(ring_a[next_i]);
            }
        }

        // Tip cap
        PackedVector3Array tip_ring = all_stations_points[all_stations_points.size() - 1];
        for (int i = 1; i < tip_ring.size() - 1; i++) {
            st->add_vertex(tip_ring[0]);
            st->add_vertex(tip_ring[i + 1]);
            st->add_vertex(tip_ring[i]);
        }
    }

    st->generate_normals();
    _mesh = st->commit();
}

PackedVector2Array AeroGeometry::get_resampled_points(const Ref<AirfoilResource> &airfoil) const {
    PackedVector2Array resampled;
    if (airfoil.is_null()) {
        for (int i = 0; i < POINTS_PER_RING; i++) {
            resampled.append(Vector2());
        }
        return resampled;
    }

    PackedVector2Array raw = airfoil->get_outline_points();
    if (raw.size() < 2) {
        for (int i = 0; i < POINTS_PER_RING; i++) {
            resampled.append(Vector2());
        }
        return resampled;
    }

    for (int i = 0; i < POINTS_PER_RING; i++) {
        double t = (double)i / (POINTS_PER_RING - 1);
        double float_idx = t * (raw.size() - 1);

        int idx_low = (int)float_idx;
        int idx_high = MIN(idx_low + 1, raw.size() - 1);
        double weight = float_idx - idx_low;

        Vector2 p = raw[idx_low].lerp(raw[idx_high], weight);
        resampled.append(p);
    }

    return resampled;
}
