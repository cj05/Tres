#include "aero_surface.hpp"
#include "part.hpp"
#include "aero/vlm_solver.hpp"
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

using namespace godot;

AeroSurface::AeroSurface() {}
AeroSurface::~AeroSurface() {}

void AeroSurface::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_debug_draw", "debug"), &AeroSurface::set_debug_draw);
    ClassDB::bind_method(D_METHOD("is_debug_draw"), &AeroSurface::is_debug_draw);
    ClassDB::bind_method(D_METHOD("set_segments_per_meter", "segments"), &AeroSurface::set_segments_per_meter);
    ClassDB::bind_method(D_METHOD("get_segments_per_meter"), &AeroSurface::get_segments_per_meter);
    ClassDB::bind_method(D_METHOD("set_wind_velocity", "wind"), &AeroSurface::set_wind_velocity);
    ClassDB::bind_method(D_METHOD("get_wind_velocity"), &AeroSurface::get_wind_velocity);
    ClassDB::bind_method(D_METHOD("set_vlm_enabled", "enabled"), &AeroSurface::set_vlm_enabled);
    ClassDB::bind_method(D_METHOD("is_vlm_enabled"), &AeroSurface::is_vlm_enabled);
    ClassDB::bind_method(D_METHOD("set_debug_force_scale", "scale"), &AeroSurface::set_debug_force_scale);
    ClassDB::bind_method(D_METHOD("get_debug_force_scale"), &AeroSurface::get_debug_force_scale);
    ClassDB::bind_method(D_METHOD("_generate_subsections"), &AeroSurface::_generate_subsections);

    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "debug_draw"), "set_debug_draw", "is_debug_draw");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "segments_per_meter"), "set_segments_per_meter", "get_segments_per_meter");
    ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "wind_velocity"), "set_wind_velocity", "get_wind_velocity");
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "vlm_enabled"), "set_vlm_enabled", "is_vlm_enabled");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "debug_force_scale"), "set_debug_force_scale", "get_debug_force_scale");
}

void AeroSurface::set_debug_draw(bool p_debug) {
    if (debug_draw == p_debug) return;
    debug_draw = p_debug;
    if (debug_mesh_instance) {
        debug_mesh_instance->set_visible(debug_draw);
    }
}

bool AeroSurface::is_debug_draw() const {
    return debug_draw;
}

void AeroSurface::set_segments_per_meter(double p_segments) {
    if (Math::is_equal_approx(segments_per_meter, p_segments)) return;
    segments_per_meter = MAX(p_segments, 0.1);
    _generate_subsections();
}

double AeroSurface::get_segments_per_meter() const {
    return segments_per_meter;
}

void AeroSurface::set_wind_velocity(Vector3 p_wind) {
    if (wind_velocity == p_wind) return;
    wind_velocity = p_wind;
}

Vector3 AeroSurface::get_wind_velocity() const {
    return wind_velocity;
}

void AeroSurface::set_vlm_enabled(bool p_enabled) {
    if (vlm_enabled == p_enabled) return;
    vlm_enabled = p_enabled;
}

bool AeroSurface::is_vlm_enabled() const {
    return vlm_enabled;
}

void AeroSurface::set_debug_force_scale(double p_scale) {
    debug_force_scale = p_scale;
}

double AeroSurface::get_debug_force_scale() const {
    return debug_force_scale;
}

void AeroSurface::_on_component_ready() {
    if (debug_mesh_instance == nullptr) {
        debug_mesh_instance = memnew(MeshInstance3D);
        debug_mesh.instantiate();
        debug_material.instantiate();
        debug_material->set_shading_mode(StandardMaterial3D::SHADING_MODE_UNSHADED);
        debug_material->set_flag(StandardMaterial3D::FLAG_ALBEDO_FROM_VERTEX_COLOR, true);
        debug_material->set_flag(StandardMaterial3D::FLAG_DISABLE_DEPTH_TEST, true);

        debug_mesh_instance->set_mesh(debug_mesh);
        debug_mesh_instance->set_material_override(debug_material);
        add_child(debug_mesh_instance);
        debug_mesh_instance->set_visible(debug_draw);
    }

    _generate_subsections();
}

void AeroSurface::_generate_subsections() {
    subsections.clear();
    Part *p = Object::cast_to<Part>(get_part());
    if (!p) return;

    AeroGeometry *geo = nullptr;
    TypedArray<ModularComponent> components = p->get_components();
    for (int i = 0; i < components.size(); i++) {
        geo = Object::cast_to<AeroGeometry>(components[i]);
        if (geo) break;
    }

    if (!geo) return;

    Ref<AeroGeometryProperties> props = geo->get_properties();
    if (props.is_valid()) {
        if (!props->is_connected("changed", Callable(this, "_generate_subsections"))) {
            props->connect("changed", Callable(this, "_generate_subsections"));
        }
    }

    if (props.is_null()) return;

    TypedArray<WingSection> sections = props->get_sections();
    
    struct Station {
        Transform3D transform;
        double chord;
        Ref<AirfoilResource> airfoil;
    };

    Vector<Station> stations;
    Station root;
    root.transform = Transform3D();
    root.chord = props->get_root_chord();
    root.airfoil = props->get_root_airfoil();
    stations.push_back(root);

    Transform3D current_tf;
    for (int i = 0; i < sections.size(); i++) {
        Ref<WingSection> s = sections[i];
        if (s.is_null()) continue;
        current_tf.basis = current_tf.basis.rotated(Vector3(1, 0, 0), Math::deg_to_rad(s->get_dihedral_angle()));
        current_tf.origin += current_tf.basis.xform(Vector3(s->get_sweep_offset(), 0, s->get_span_offset()));
        Station st;
        st.transform = current_tf;
        st.chord = s->get_chord();
        st.airfoil = s->get_airfoil_data();
        stations.push_back(st);
    }

    if (stations.size() < 2) return;

    for (int i = 0; i < stations.size() - 1; i++) {
        Station s1 = stations[i];
        Station s2 = stations[i+1];
        double segment_span = s1.transform.origin.distance_to(s2.transform.origin);
        int num_sub = (int)Math::round(segment_span * segments_per_meter);
        if (num_sub < 1) num_sub = 1;
        for (int j = 0; j < num_sub; j++) {
            double t1 = (double)j / num_sub;
            double t2 = (double)(j + 1) / num_sub;
            double cos_t1 = (1.0 - Math::cos(t1 * Math_PI)) * 0.5;
            double cos_t2 = (1.0 - Math::cos(t2 * Math_PI)) * 0.5;
            double cos_mid = (1.0 - Math::cos((t1 + t2) * 0.5 * Math_PI)) * 0.5;
            SubSection sub;
            sub.transform.origin = s1.transform.origin.lerp(s2.transform.origin, cos_mid);
            sub.transform.basis = s1.transform.basis.slerp(s2.transform.basis, cos_mid);
            sub.chord = Math::lerp(s1.chord, s2.chord, cos_mid);
            sub.airfoil = (cos_mid < 0.5) ? s1.airfoil : s2.airfoil;
            double sub_span = segment_span * (cos_t2 - cos_t1);
            sub.area = sub_span * sub.chord;
            subsections.push_back(sub);
        }
    }
}

void AeroSurface::_update_vortices() {
    vortices.clear();
    for (int i = 0; i < subsections.size(); i++) {
        const SubSection &sub = subsections[i];
        Vortex v;
        double panel_span = sub.area / sub.chord;
        Vector3 local_left_tip(0.25 * sub.chord, 0, -0.5 * panel_span);
        Vector3 local_right_tip(0.25 * sub.chord, 0, 0.5 * panel_span);
        Vector3 local_collocation(0.75 * sub.chord, 0, 0);
        Vector3 local_normal(0, 1, 0);
        v.left_tip = sub.transform.xform(local_left_tip);
        v.right_tip = sub.transform.xform(local_right_tip);
        v.collocation_point = sub.transform.xform(local_collocation);
        v.normal = sub.transform.basis.xform(local_normal).normalized();
        v.span = panel_span;
        vortices.push_back(v);
    }
}

void AeroSurface::_solve_vlm() {
    if (vortices.size() == 0) return;
    
    Vector<VLMPanel> panels;
    for (int i = 0; i < vortices.size(); i++) {
        const Vortex &v = vortices[i];
        VLMPanel p;
        p.left_tip = v.left_tip;
        p.right_tip = v.right_tip;
        p.collocation_point = v.collocation_point;
        p.normal = v.normal;
        p.area = subsections[i].area;
        p.chord = subsections[i].chord;
        panels.push_back(p);
    }

    Vector3 local_wind_node = get_global_transform().basis.xform_inv(wind_velocity);
    VLMSolver::solve(panels, local_wind_node);

    for (int i = 0; i < vortices.size(); i++) {
        vortices.write[i].circulation = panels[i].circulation;
    }
}

void AeroSurface::_update_debug_draw() {
    if (debug_mesh.is_null() || !debug_draw) return;
    debug_mesh->clear_surfaces();
    if (subsections.size() == 0) return;
    debug_mesh->surface_begin(Mesh::PRIMITIVE_LINES);
    
    Vector3 local_wind_node = get_global_transform().basis.xform_inv(wind_velocity);
    Vector3 wind_dir = local_wind_node.normalized();

    for (int i = 0; i < subsections.size(); i++) {
        const SubSection &sub = subsections[i];
        debug_mesh->surface_set_color(Color(0.3, 0.3, 0.3));
        debug_mesh->surface_add_vertex(sub.transform.origin);
        debug_mesh->surface_add_vertex(sub.transform.origin + sub.transform.basis.xform(Vector3(sub.chord, 0, 0)));

        if (vlm_enabled && i < vortices.size()) {
            const Vortex &v = vortices[i];
            debug_mesh->surface_set_color(Color(0.2, 0.2, 1.0));
            debug_mesh->surface_add_vertex(v.left_tip);
            debug_mesh->surface_add_vertex(v.right_tip);
            debug_mesh->surface_set_color(Color(0.1, 0.1, 0.5));
            debug_mesh->surface_add_vertex(v.left_tip);
            debug_mesh->surface_add_vertex(v.left_tip + wind_dir * debug_vortex_scale);
            debug_mesh->surface_add_vertex(v.right_tip);
            debug_mesh->surface_add_vertex(v.right_tip + wind_dir * debug_vortex_scale);

            // Lift from VLM
            debug_mesh->surface_set_color(Color(0, 1.0, 1.0)); 
            debug_mesh->surface_add_vertex(sub.transform.origin);
            debug_mesh->surface_add_vertex(sub.transform.origin + v.lift_vector * debug_force_scale);
        } else {
            // Lift and Drag from strip theory
            debug_mesh->surface_set_color(Color(0, 1.0, 1.0)); 
            debug_mesh->surface_add_vertex(sub.transform.origin);
            debug_mesh->surface_add_vertex(sub.transform.origin + sub.lift_vector * debug_force_scale);

            debug_mesh->surface_set_color(Color(1.0, 0.2, 0.2)); 
            debug_mesh->surface_add_vertex(sub.transform.origin);
            debug_mesh->surface_add_vertex(sub.transform.origin + sub.drag_vector * debug_force_scale);
        }
    }
    debug_mesh->surface_end();
}

void AeroSurface::physics_step(Variant p_state) {}

Vector3 AeroSurface::compute_force(Variant p_state) {
    Vector3 total_force_local; // Local to AeroSurface
    double rho = 1.225;
    Vector3 local_wind_node = get_global_transform().basis.xform_inv(wind_velocity);
    double speed = local_wind_node.length();

    if (vlm_enabled) {
        _update_vortices();
        _solve_vlm();
        for (int i = 0; i < vortices.size(); i++) {
            Vortex &v = vortices.write[i];
            Vector3 bound_vec = v.right_tip - v.left_tip;
            
            // Formula: L = rho * (V x Gamma_vec)
            // My sign convention: local_wind_node is V, bound_vec is Gamma direction.
            // (1, 0, 0) x (0, 0, 1) = (0, -1, 0).
            // Since G is negative for lift, this produces (0, 1, 0). Correct.
            Vector3 force = local_wind_node.cross(bound_vec) * (rho * v.circulation);
            
            v.lift_vector = force;
            v.lift = force.dot(v.normal);
            v.cl = (speed > 0.1) ? (2.0 * v.circulation) / (speed * subsections[i].chord) : 0.0;
            total_force_local += force;
        }
    } else {
        for (int i = 0; i < subsections.size(); i++) {
            SubSection &sub = subsections.write[i];
            Vector3 local_vel = sub.transform.basis.xform_inv(local_wind_node);
            double speed_sub = local_vel.length();
            if (speed_sub < 0.1) { sub.lift_vector = Vector3(); sub.drag_vector = Vector3(); continue; }

            double alpha = Math::atan2(-local_vel.y, -local_vel.x);
            AeroInput in; in.rho = rho; in.speed = speed_sub; in.alpha = alpha; in.area = sub.area; in.chord = sub.chord;
            AeroOutput out = flatplate_compute(in);
            
            Vector3 local_wind_dir = local_vel.normalized();
            Vector3 local_lift_dir = Vector3(-local_wind_dir.y, local_wind_dir.x, 0).normalized();
            
            sub.lift_vector = sub.transform.basis.xform(local_lift_dir * out.lift);
            sub.drag_vector = sub.transform.basis.xform(-local_wind_dir * out.drag);
            
            total_force_local += sub.lift_vector + sub.drag_vector;
        }
    }

    // Return force in PARENT space (the Part node)
    return get_transform().basis.xform(total_force_local);
}

void AeroSurface::_process(double delta) {
    if (Engine::get_singleton()->is_editor_hint() || is_inside_tree()) {
        compute_force(Variant());
        _update_debug_draw();
    }
}
