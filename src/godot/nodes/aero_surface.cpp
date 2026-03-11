#include "aero_surface.hpp"
#include "../core/part.hpp"
#include "../utils/wing_generator.hpp"
#include "aero/models/vlm_model.hpp"
#include "aero/models/flatplate_model.hpp"
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

using namespace godot;

// Conversion helpers
static aero::Vector3 to_aero(const Vector3& v) { return aero::Vector3(v.x, v.y, v.z); }
static Vector3 from_aero(const aero::Vector3& v) { return Vector3((double)v.x, (double)v.y, (double)v.z); }

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
    if (debug_mesh_instance) debug_mesh_instance->set_visible(debug_draw);
}

bool AeroSurface::is_debug_draw() const { return debug_draw; }

void AeroSurface::set_segments_per_meter(double p_segments) {
    if (Math::is_equal_approx(segments_per_meter, p_segments)) return;
    segments_per_meter = MAX(p_segments, 0.1);
    dirty = true;
    _generate_subsections();
}

double AeroSurface::get_segments_per_meter() const { return segments_per_meter; }

void AeroSurface::set_wind_velocity(Vector3 p_wind) {
    if (wind_velocity == p_wind) return;
    wind_velocity = p_wind;
    dirty = true;
}
Vector3 AeroSurface::get_wind_velocity() const { return wind_velocity; }

void AeroSurface::set_vlm_enabled(bool p_enabled) {
    if (vlm_enabled == p_enabled) return;
    vlm_enabled = p_enabled;
    dirty = true;
}
bool AeroSurface::is_vlm_enabled() const { return vlm_enabled; }

void AeroSurface::set_debug_force_scale(double p_scale) { debug_force_scale = p_scale; }
double AeroSurface::get_debug_force_scale() const { return debug_force_scale; }

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
    if (props.is_null()) return;

    if (!props->is_connected("changed", Callable(this, "_generate_subsections")))
        props->connect("changed", Callable(this, "_generate_subsections"));

    Vector<WingStation> stations = WingGenerator::generate_stations(props);
    Vector<WingSubsection> wing_subs = WingGenerator::generate_subsections(stations, segments_per_meter);
    
    for (const auto& ws : wing_subs) {
        SubSection s;
        s.transform = ws.transform;
        s.area = ws.area;
        s.chord = ws.chord;
        s.airfoil = ws.airfoil;
        subsections.push_back(s);
    }
    dirty = true;
}

void AeroSurface::_update_vortices() {
    vortices.clear();
    for (int i = 0; i < subsections.size(); i++) {
        const SubSection &sub = subsections[i];
        Vortex v;
        double panel_span = sub.area / sub.chord;
        
        // Define tips in local subsection space
        Vector3 p_left = Vector3(0.25 * sub.chord, 0, -0.5 * panel_span);
        Vector3 p_right = Vector3(0.25 * sub.chord, 0, 0.5 * panel_span);
        
        v.left_tip = sub.transform.xform(p_left);
        v.right_tip = sub.transform.xform(p_right);
        v.collocation_point = sub.transform.xform(Vector3(0.75 * sub.chord, 0, 0));
        v.normal = sub.transform.basis.xform(Vector3(0, 1, 0)).normalized();
        v.span = panel_span;

        // Ensure tips are ordered correctly relative to the normal (Right-Hand Rule)
        // This is crucial for mirrored wings where the basis handedness is flipped.
        Vector3 calculated_normal = (v.right_tip - v.left_tip).cross(v.collocation_point - v.left_tip);
        if (calculated_normal.dot(v.normal) < 0) {
            UtilityFunctions::print("AeroSurface Inverted, flipping points");
            Vector3 temp = v.left_tip;
            v.left_tip = v.right_tip;
            v.right_tip = temp;
        }

        vortices.push_back(v);
    }
}

void AeroSurface::_solve_vlm() {
    if (vortices.size() == 0) return;
    aero::Vector<aero::VLMPanel> panels;
    for (int i = 0; i < vortices.size(); i++) {
        aero::VLMPanel p;
        p.left_tip = to_aero(vortices[i].left_tip);
        p.right_tip = to_aero(vortices[i].right_tip);
        p.collocation_point = to_aero(vortices[i].collocation_point);
        p.normal = to_aero(vortices[i].normal);
        p.area = subsections[i].area;
        p.chord = subsections[i].chord;
        panels.push_back(p);
    }
    Vector3 local_wind_node = get_global_transform().basis.xform_inv(wind_velocity);
    aero::VLMModel::solve(panels, to_aero(local_wind_node));

    if (panels.size() > 1) {
        UtilityFunctions::print("AeroSurface [", get_name(), "] VLM Solve Results:");
        UtilityFunctions::print("  - Panel 0 Gamma: ", panels[0].circulation);
        UtilityFunctions::print("  - Panel N Gamma: ", panels[panels.size()-1].circulation);
        UtilityFunctions::print("  - Local Wind: ", local_wind_node);
    }

    

    for (int i = 0; i < vortices.size(); i++) vortices.write[i].circulation = (double)panels[i].circulation;

    if (debug_draw) {
        UtilityFunctions::print("--- VLM Circulation Map [", get_name(), "] ---");
        for (int i = 0; i < vortices.size(); i++) {
            UtilityFunctions::print("  Panel ", i, ": ", vortices[i].circulation);
        }
        UtilityFunctions::print("------------------------------------------");
    }
}

void AeroSurface::_update_debug_draw() {
    if (debug_mesh.is_null() || !debug_draw) return;
    debug_mesh->clear_surfaces();
    if (subsections.size() == 0) return;

    debug_mesh->surface_begin(Mesh::PRIMITIVE_LINES);
    Vector3 local_wind_node = get_global_transform().basis.xform_inv(wind_velocity);
    Vector3 wind_dir = local_wind_node.normalized();
    
    // Find max circulation for normalized coloring
    double max_gamma = 0.01;
    if (vlm_enabled) {
        for (int i = 0; i < vortices.size(); i++) {
            max_gamma = MAX(max_gamma, Math::abs(vortices[i].circulation));
        }
    }

    for (int i = 0; i < subsections.size(); i++) {
        const SubSection &sub = subsections[i];
        
        // 1. Draw Chord Line (Gray)
        debug_mesh->surface_set_color(Color(0.4, 0.4, 0.4));
        debug_mesh->surface_add_vertex(sub.transform.origin);
        debug_mesh->surface_add_vertex(sub.transform.origin + sub.transform.basis.xform(Vector3(sub.chord, 0, 0)));

        if (vlm_enabled && i < vortices.size()) {
            const Vortex &v = vortices[i];
            double gamma_ratio = Math::abs(v.circulation) / max_gamma;
            
            // 2. Bound Segment (Color based on Gamma: Blue -> Pink/White)
            Color gamma_color = Color(0.2, 0.5, 1.0).lerp(Color(1.0, 0.2, 0.8), gamma_ratio);
            debug_mesh->surface_set_color(gamma_color);
            debug_mesh->surface_add_vertex(v.left_tip);
            debug_mesh->surface_add_vertex(v.right_tip);

            // 3. Horseshoe Trailing Legs (Fading Blue)
            debug_mesh->surface_set_color(Color(0.1, 0.1, 0.5, 0.5));
            debug_mesh->surface_add_vertex(v.left_tip);
            debug_mesh->surface_add_vertex(v.left_tip + wind_dir * debug_vortex_scale * 10.0);
            debug_mesh->surface_add_vertex(v.right_tip);
            debug_mesh->surface_add_vertex(v.right_tip + wind_dir * debug_vortex_scale * 10.0);

            // 4. Lift Vector (Cyan)
            debug_mesh->surface_set_color(Color(0, 1.0, 1.0));
            debug_mesh->surface_add_vertex(sub.transform.origin);
            debug_mesh->surface_add_vertex(sub.transform.origin + v.lift_vector * debug_force_scale);
            
            // 5. Collocation Point (Small cross)
            debug_mesh->surface_set_color(Color(1, 1, 0)); // Yellow
            Vector3 cp = v.collocation_point;
            double s = 0.02;
            debug_mesh->surface_add_vertex(cp + Vector3(s,0,0)); debug_mesh->surface_add_vertex(cp - Vector3(s,0,0));
            debug_mesh->surface_add_vertex(cp + Vector3(0,s,0)); debug_mesh->surface_add_vertex(cp - Vector3(0,s,0));
            debug_mesh->surface_add_vertex(cp + Vector3(0,0,s)); debug_mesh->surface_add_vertex(cp - Vector3(0,0,s));

        } else {
            // 6. Non-VLM Lift (Cyan) and Drag (Red)
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
    Transform3D current_transform = get_global_transform();
    Vector3 local_wind_node = current_transform.basis.xform_inv(wind_velocity);

    // Check if we can use cached results
    if (!dirty && 
        local_wind_node.is_equal_approx(last_local_wind) && 
        current_transform.is_equal_approx(last_transform)) {
        return current_transform.basis.xform(force_cache);
    }

    Vector3 total_force_local;
    double rho = 1.225;
    double speed = local_wind_node.length();

    if (vlm_enabled) {
        _update_vortices();
        _solve_vlm();

        for (int i = 0; i < vortices.size(); i++) {
            Vortex &v = vortices.write[i];
            Vector3 bound_vector = (v.right_tip - v.left_tip);
            Vector3 force = bound_vector.cross(local_wind_node) * (rho * v.circulation);
            
            v.lift_vector = force;
            v.lift = force.dot(v.normal);
            
            // Cl calculation: (2 * L) / (rho * v^2 * S)
            double dyn_press = 0.5 * rho * speed * speed;
            v.cl = (speed > 0.1) ? v.lift / (dyn_press * subsections[i].area) : 0.0;
            
            total_force_local += force;
        }
    } else {
        aero::FlatPlateModel model;
        for (int i = 0; i < subsections.size(); i++) {
            SubSection &sub = subsections.write[i];
            Vector3 local_vel = sub.transform.basis.xform_inv(local_wind_node);
            double s_sub = local_vel.length();
            if (s_sub < 0.1) {
                sub.lift_vector = sub.drag_vector = Vector3();
                continue;
            }
            double alpha = Math::atan2(-local_vel.y, -local_vel.x);
            aero::AeroInput in;
            in.rho = rho;
            in.speed = s_sub;
            in.alpha = alpha;
            in.area = sub.area;
            in.chord = sub.chord;
            aero::AeroOutput out = model.compute(in);
            Vector3 l_dir = Vector3(-local_vel.y, local_vel.x, 0).normalized();
            sub.lift_vector = sub.transform.basis.xform(l_dir * out.lift);
            sub.drag_vector = sub.transform.basis.xform(-local_vel.normalized() * out.drag);
            total_force_local += sub.lift_vector + sub.drag_vector;
        }
    }

    // Update cache
    force_cache = total_force_local;
    last_local_wind = local_wind_node;
    last_transform = current_transform;
    dirty = false;

    return current_transform.basis.xform(total_force_local);
}

void AeroSurface::_process(double delta) {
    if (Engine::get_singleton()->is_editor_hint() || is_inside_tree()) {
        compute_force(Variant()); _update_debug_draw();
    }
}
