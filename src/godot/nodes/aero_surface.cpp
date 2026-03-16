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
    ClassDB::bind_method(D_METHOD("set_debug_solve_results", "enabled"), &AeroSurface::set_debug_solve_results);
    ClassDB::bind_method(D_METHOD("is_debug_solve_results"), &AeroSurface::is_debug_solve_results);
    ClassDB::bind_method(D_METHOD("set_debug_influence_draw", "enabled"), &AeroSurface::set_debug_influence_draw);
    ClassDB::bind_method(D_METHOD("is_debug_influence_draw"), &AeroSurface::is_debug_influence_draw);
    ClassDB::bind_method(D_METHOD("set_vortex_threshold_factor", "factor"), &AeroSurface::set_vortex_threshold_factor);
    ClassDB::bind_method(D_METHOD("get_vortex_threshold_factor"), &AeroSurface::get_vortex_threshold_factor);
    ClassDB::bind_method(D_METHOD("set_segments_per_meter", "segments"), &AeroSurface::set_segments_per_meter);
    ClassDB::bind_method(D_METHOD("get_segments_per_meter"), &AeroSurface::get_segments_per_meter);
    ClassDB::bind_method(D_METHOD("set_use_cosine_spacing", "enabled"), &AeroSurface::set_use_cosine_spacing);
    ClassDB::bind_method(D_METHOD("get_use_cosine_spacing"), &AeroSurface::get_use_cosine_spacing);
    ClassDB::bind_method(D_METHOD("set_wind_velocity", "wind"), &AeroSurface::set_wind_velocity);
    ClassDB::bind_method(D_METHOD("get_wind_velocity"), &AeroSurface::get_wind_velocity);
    ClassDB::bind_method(D_METHOD("set_vlm_enabled", "enabled"), &AeroSurface::set_vlm_enabled);
    ClassDB::bind_method(D_METHOD("is_vlm_enabled"), &AeroSurface::is_vlm_enabled);
    ClassDB::bind_method(D_METHOD("set_dynamic_stall_enabled", "enabled"), &AeroSurface::set_dynamic_stall_enabled);
    ClassDB::bind_method(D_METHOD("is_dynamic_stall_enabled"), &AeroSurface::is_dynamic_stall_enabled);
    ClassDB::bind_method(D_METHOD("set_debug_force_scale", "scale"), &AeroSurface::set_debug_force_scale);
    ClassDB::bind_method(D_METHOD("get_debug_force_scale"), &AeroSurface::get_debug_force_scale);
    ClassDB::bind_method(D_METHOD("get_force_cache"), &AeroSurface::get_force_cache);
    ClassDB::bind_method(D_METHOD("get_aoa"), &AeroSurface::get_aoa);
    ClassDB::bind_method(D_METHOD("_generate_subsections"), &AeroSurface::_generate_subsections);

    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "debug_draw"), "set_debug_draw", "is_debug_draw");
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "debug_solve_results"), "set_debug_solve_results", "is_debug_solve_results");
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "debug_influence_draw"), "set_debug_influence_draw", "is_debug_influence_draw");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "vortex_threshold_factor"), "set_vortex_threshold_factor", "get_vortex_threshold_factor");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "segments_per_meter"), "set_segments_per_meter", "get_segments_per_meter");
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "use_cosine_spacing"), "set_use_cosine_spacing", "get_use_cosine_spacing");
    ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "wind_velocity"), "set_wind_velocity", "get_wind_velocity");
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "vlm_enabled"), "set_vlm_enabled", "is_vlm_enabled");
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "dynamic_stall_enabled"), "set_dynamic_stall_enabled", "is_dynamic_stall_enabled");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "debug_force_scale"), "set_debug_force_scale", "get_debug_force_scale");
    ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "current_force", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_EDITOR | PROPERTY_USAGE_READ_ONLY), "", "get_force_cache");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "current_aoa", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_EDITOR | PROPERTY_USAGE_READ_ONLY), "", "get_aoa");
}

void AeroSurface::set_debug_draw(bool p_debug) {
    if (debug_draw == p_debug) return;
    debug_draw = p_debug;
    if (debug_mesh_instance) debug_mesh_instance->set_visible(debug_draw);
}

bool AeroSurface::is_debug_draw() const { return debug_draw; }

void AeroSurface::set_debug_solve_results(bool p_enabled) { debug_solve_results = p_enabled; }
bool AeroSurface::is_debug_solve_results() const { return debug_solve_results; }

void AeroSurface::set_debug_influence_draw(bool p_enabled) { debug_influence_draw = p_enabled; }
bool AeroSurface::is_debug_influence_draw() const { return debug_influence_draw; }

void AeroSurface::set_vortex_threshold_factor(double p_factor) { vortex_threshold_factor = p_factor; }
double AeroSurface::get_vortex_threshold_factor() const { return vortex_threshold_factor; }

void AeroSurface::set_segments_per_meter(double p_segments) {
    if (Math::is_equal_approx(segments_per_meter, p_segments)) return;
    segments_per_meter = MAX(p_segments, 0.1);
    dirty = true;
    _generate_subsections();
}

double AeroSurface::get_segments_per_meter() const { return segments_per_meter; }

void AeroSurface::set_use_cosine_spacing(bool p_enabled) {
    if (use_cosine_spacing == p_enabled) return;
    use_cosine_spacing = p_enabled;
    dirty = true;
    _generate_subsections();
}

bool AeroSurface::get_use_cosine_spacing() const { return use_cosine_spacing; }

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

void AeroSurface::set_dynamic_stall_enabled(bool p_enabled) {
    if (dynamic_stall_enabled == p_enabled) return;
    dynamic_stall_enabled = p_enabled;
    dirty = true;
}
bool AeroSurface::is_dynamic_stall_enabled() const { return dynamic_stall_enabled; }

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
    Vector<WingSubsection> wing_subs = WingGenerator::generate_subsections(stations, segments_per_meter, use_cosine_spacing);
    
    for (const auto& ws : wing_subs) {
        SubSection s;
        s.transform = ws.transform;
        s.area = ws.area;
        s.chord = ws.chord;
        s.span = ws.span;
        s.airfoil = ws.airfoil;
        s.v1_4_left = ws.v1_4_left;
        s.v1_4_right = ws.v1_4_right;
        s.v3_4_left = ws.v3_4_left;
        s.v3_4_right = ws.v3_4_right;
        subsections.push_back(s);
    }
    dirty = true;
}

void AeroSurface::_update_vortices() {
    vortices.clear();
    Part *p = Object::cast_to<Part>(get_part());
    if (!p) return;

    Transform3D part_to_local = get_global_transform().affine_inverse() * p->get_global_transform();

    for (int i = 0; i < subsections.size(); i++) {
        const SubSection &sub = subsections[i];
        Vortex v;

        v.left_tip = part_to_local.xform(sub.v1_4_left);
        v.right_tip = part_to_local.xform(sub.v1_4_right);

        Vector3 c_left = part_to_local.xform(sub.v3_4_left);
        Vector3 c_right = part_to_local.xform(sub.v3_4_right);
        v.collocation_point = (c_left + c_right) * 0.5;

        Vector3 local_up = part_to_local.basis.xform(sub.transform.basis.get_column(1)).normalized();
        Vector3 mid_1_4 = (v.left_tip + v.right_tip) * 0.5;
        v.normal = (v.right_tip - v.left_tip).cross(v.collocation_point - mid_1_4).normalized();

        if (v.normal.dot(local_up) < 0) {
            v.normal = -v.normal;
        }

        v.span = sub.area / sub.chord;
        v.stall_model = std::make_unique<aero::CirculationLagModel>(2.0);
        vortices.push_back(std::move(v));
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
    aero::VLMModel::solve(panels, to_aero(local_wind_node), debug_solve_results);

    if (debug_solve_results && panels.size() > 1) {
        UtilityFunctions::print("AeroSurface [", get_name(), "] VLM Solve Results:");
        UtilityFunctions::print("  - Panel 0 Gamma: ", panels[0].circulation);
        UtilityFunctions::print("  - Panel N Gamma: ", panels[panels.size()-1].circulation);
        UtilityFunctions::print("  - Local Wind: ", local_wind_node);
    }

    for (int i = 0; i < vortices.size(); i++) vortices.write[i].circulation = (double)panels[i].circulation;
}

void AeroSurface::_analyze_wake() {
    trailing_filaments.clear();
    wake_clusters.clear();
    if (vortices.size() == 0) return;

    // Leg 0: Left tip of panel 0
    TrailingFilament f_start;
    f_start.pos = vortices[0].left_tip;
    f_start.strength = vortices[0].circulation;
    aero::VLMPanel p0;
    p0.left_tip = to_aero(vortices[0].left_tip); p0.right_tip = to_aero(vortices[0].right_tip); p0.collocation_point = to_aero(vortices[0].collocation_point);
    f_start.direction = from_aero(aero::VLMModel::get_panel_wake_dir(p0));
    trailing_filaments.push_back(f_start);

    for (int i = 0; i < vortices.size() - 1; i++) {
        TrailingFilament f;
        f.pos = (vortices[i].right_tip + vortices[i+1].left_tip) * 0.5;
        f.strength = vortices[i+1].circulation - vortices[i].circulation;
        aero::VLMPanel p1, p2;
        p1.left_tip = to_aero(vortices[i].left_tip); p1.right_tip = to_aero(vortices[i].right_tip); p1.collocation_point = to_aero(vortices[i].collocation_point);
        p2.left_tip = to_aero(vortices[i+1].left_tip); p2.right_tip = to_aero(vortices[i+1].right_tip); p2.collocation_point = to_aero(vortices[i+1].collocation_point);
        f.direction = (from_aero(aero::VLMModel::get_panel_wake_dir(p1)) + from_aero(aero::VLMModel::get_panel_wake_dir(p2))).normalized();
        trailing_filaments.push_back(f);
    }

    TrailingFilament f_end;
    f_end.pos = vortices[vortices.size() - 1].right_tip;
    f_end.strength = -vortices[vortices.size() - 1].circulation;
    aero::VLMPanel pN;
    pN.left_tip = to_aero(vortices[vortices.size()-1].left_tip); pN.right_tip = to_aero(vortices[vortices.size()-1].right_tip); pN.collocation_point = to_aero(vortices[vortices.size()-1].collocation_point);
    f_end.direction = from_aero(aero::VLMModel::get_panel_wake_dir(pN));
    trailing_filaments.push_back(f_end);

    double max_shed = 0.001;
    for (const auto& f : trailing_filaments) max_shed = MAX(max_shed, Math::abs(f.strength));
    double threshold = vortex_threshold_factor * max_shed;

    int current_start = -1;
    for (int i = 0; i < trailing_filaments.size(); i++) {
        bool is_candidate = Math::abs(trailing_filaments[i].strength) > threshold;
        if (is_candidate) {
            if (current_start == -1) current_start = i;
        } else if (current_start != -1) {
            VortexStructure vs;
            vs.segment_start = current_start; vs.segment_end = i - 1;
            Vector3 sum_pos; double sum_strength = 0; vs.bounding_box = AABB(trailing_filaments[current_start].pos, Vector3());
            Vector3 avg_dir;
            for (int k = current_start; k < i; k++) {
                double s = trailing_filaments[k].strength;
                sum_pos += trailing_filaments[k].pos * Math::abs(s);
                sum_strength += s;
                vs.bounding_box.expand_to(trailing_filaments[k].pos);
                avg_dir += trailing_filaments[k].direction;
            }
            vs.strength = sum_strength;
            vs.center = sum_pos / MAX(1e-6, Math::abs(sum_strength));
            vs.direction = avg_dir.normalized();
            vs.bounding_box.expand_to(vs.bounding_box.position + vs.direction * 2.0);
            wake_clusters.push_back(vs);
            current_start = -1;
        }
    }
}

void AeroSurface::_update_debug_draw() {
    if (debug_mesh.is_null() || !debug_draw) return;
    debug_mesh->clear_surfaces();
    if (subsections.size() == 0) return;

    Part *p = Object::cast_to<Part>(get_part());
    if (!p) return;

    Transform3D part_to_local = get_global_transform().affine_inverse() * p->get_global_transform();
    debug_mesh->surface_begin(Mesh::PRIMITIVE_LINES);
    Vector3 local_wind_node = get_global_transform().basis.xform_inv(wind_velocity);
    
    double max_gamma = 0.01;
    if (vlm_enabled) {
        for (int i = 0; i < vortices.size(); i++) max_gamma = MAX(max_gamma, Math::abs(vortices[i].circulation));
    }

    for (int i = 0; i < subsections.size(); i++) {
        const SubSection &sub = subsections[i];
        Vector3 sub_origin_local = part_to_local.xform(sub.transform.origin);
        debug_mesh->surface_set_color(Color(0.4, 0.4, 0.4));
        debug_mesh->surface_add_vertex(sub_origin_local);
        debug_mesh->surface_add_vertex(sub_origin_local + part_to_local.basis.xform(sub.transform.basis.xform(Vector3(sub.chord, 0, 0))));

        if (vlm_enabled && i < vortices.size()) {
            const Vortex &v = vortices[i];
            double gamma_ratio = Math::abs(v.circulation) / max_gamma;
            debug_mesh->surface_set_color(Color(0.2, 0.5, 1.0).lerp(Color(1.0, 0.2, 0.8), gamma_ratio));
            debug_mesh->surface_add_vertex(v.left_tip);
            debug_mesh->surface_add_vertex(v.right_tip);

            if (!debug_influence_draw) {
                aero::VLMPanel pj; pj.left_tip = to_aero(v.left_tip); pj.right_tip = to_aero(v.right_tip); pj.collocation_point = to_aero(v.collocation_point);
                Vector3 local_chord_dir = from_aero(aero::VLMModel::get_panel_wake_dir(pj));
                debug_mesh->surface_set_color(Color(0.1, 0.1, 0.5, 0.5));
                debug_mesh->surface_add_vertex(v.left_tip); debug_mesh->surface_add_vertex(v.left_tip + local_chord_dir * debug_vortex_scale * 10.0);
                debug_mesh->surface_add_vertex(v.right_tip); debug_mesh->surface_add_vertex(v.right_tip + local_chord_dir * debug_vortex_scale * 10.0);
            }

            debug_mesh->surface_set_color(Color(0, 1.0, 1.0));
            debug_mesh->surface_add_vertex(sub_origin_local);
            debug_mesh->surface_add_vertex(sub_origin_local + (v.lift_vector / v.span) * debug_force_scale);
            
            debug_mesh->surface_set_color(Color(1, 1, 0));
            Vector3 cp = v.collocation_point; double s = 0.02;
            debug_mesh->surface_add_vertex(cp + Vector3(s,0,0)); debug_mesh->surface_add_vertex(cp - Vector3(s,0,0));
            debug_mesh->surface_add_vertex(cp + Vector3(0,s,0)); debug_mesh->surface_add_vertex(cp - Vector3(0,s,0));
            debug_mesh->surface_add_vertex(cp + Vector3(0,0,s)); debug_mesh->surface_add_vertex(cp - Vector3(0,0,s));
        }
    }

    if (vlm_enabled && debug_influence_draw) {
        for (const auto& f : trailing_filaments) {
            debug_mesh->surface_set_color(Color(0.3, 0.3, 0.6, 0.3));
            debug_mesh->surface_add_vertex(f.pos); debug_mesh->surface_add_vertex(f.pos + f.direction * 5.0);
        }
        for (const auto& cluster : wake_clusters) {
            debug_mesh->surface_set_color((cluster.strength > 0) ? Color(1, 0.5, 0.2) : Color(0.2, 0.5, 1));
            for (int k = cluster.segment_start; k <= cluster.segment_end; k++) {
                debug_mesh->surface_add_vertex(trailing_filaments[k].pos); debug_mesh->surface_add_vertex(trailing_filaments[k].pos + trailing_filaments[k].direction * 5.0);
            }
            debug_mesh->surface_set_color(Color(1, 1, 1, 0.4));
            AABB b = cluster.bounding_box;
            Vector3 vertices[8] = { b.position, b.position + Vector3(b.size.x, 0, 0), b.position + Vector3(b.size.x, b.size.y, 0), b.position + Vector3(0, b.size.y, 0), b.position + Vector3(0, 0, b.size.z), b.position + Vector3(b.size.x, 0, b.size.z), b.position + b.size, b.position + Vector3(0, b.size.y, b.size.z) };
            int indices[24] = {0,1,1,2,2,3,3,0, 4,5,5,6,6,7,7,4, 0,4,1,5,2,6,3,7};
            for (int j = 0; j < 24; j++) debug_mesh->surface_add_vertex(vertices[indices[j]]);
        }
    }
    debug_mesh->surface_end();
}

void AeroSurface::physics_step(Variant p_state) {}

Vector3 AeroSurface::compute_force(Variant p_state) {
    Transform3D current_transform = get_global_transform();
    Vector3 local_wind_node = current_transform.basis.xform_inv(wind_velocity);
    aoa_cache = aero::compute_aoa(to_aero(local_wind_node));
    double dt = 1.0 / 60.0;
    if (p_state.get_type() == Variant::FLOAT) dt = p_state;
    else if (p_state.get_type() == Variant::DICTIONARY) { Dictionary d = p_state; if (d.has("dt")) dt = d["dt"]; }

    if (!dynamic_stall_enabled && !dirty && local_wind_node.is_equal_approx(last_local_wind) && current_transform.is_equal_approx(last_transform)) {
        return current_transform.basis.xform(force_cache);
    }

    Vector3 total_force_local;
    double rho = 1.225;
    double speed = local_wind_node.length();

    if (vlm_enabled) {
        if (dirty || vortices.size() == 0) _update_vortices();
        _solve_vlm();

        for (int i = 0; i < vortices.size(); i++) {
            Vortex &v = vortices.write[i];
            double cl_vlm = (speed > 0.1) ? (2.0 * v.circulation) / (speed * subsections[i].chord) : 0.0;
            v.cl_vlm = cl_vlm;

            Part *p = Object::cast_to<Part>(get_part());
            Transform3D part_to_local = current_transform.affine_inverse() * p->get_global_transform();
            Transform3D sub_tf_local = part_to_local * subsections[i].transform;
            Vector3 sub_wind = sub_tf_local.basis.xform_inv(local_wind_node);
            double alpha = aero::compute_aoa(to_aero(sub_wind));
            v.alpha = alpha;

            if (dynamic_stall_enabled && v.stall_model) {
                aero::StallInput in; in.alpha = alpha; in.alpha_dot = (alpha - v.last_alpha) / dt; in.velocity = speed; in.chord = subsections[i].chord; in.cl_vlm = cl_vlm; in.dt = dt;
                aero::StallOutput out = v.stall_model->update(in);
                v.cl = out.cl_corrected; v.last_alpha = alpha;
                v.circulation = (v.cl * speed * subsections[i].chord) / 2.0;
            } else {
                v.cl = cl_vlm;
            }

            Vector3 force = local_wind_node.cross(v.left_tip - v.right_tip) * (rho * v.circulation);
            v.lift_vector = force; v.lift = force.dot(v.normal);
            total_force_local += force;
        }
        _analyze_wake();
    } else {
        aero::FlatPlateModel model;
        for (int i = 0; i < subsections.size(); i++) {
            SubSection &sub = subsections.write[i];
            Vector3 local_vel = sub.transform.basis.xform_inv(local_wind_node);
            double s_sub = local_vel.length();
            if (s_sub < 0.1) { sub.lift_vector = sub.drag_vector = Vector3(); continue; }
            double alpha = aero::compute_aoa(to_aero(local_vel));
            sub.alpha = alpha;
            aero::AeroInput in; in.rho = rho; in.speed = s_sub; in.alpha = alpha; in.area = sub.area; in.chord = sub.chord;
            aero::AeroOutput out = model.compute(in);
            sub.lift_vector = sub.transform.basis.xform(Vector3(-local_vel.y, local_vel.x, 0).normalized() * out.lift);
            sub.drag_vector = sub.transform.basis.xform(-local_vel.normalized() * out.drag);
            total_force_local += sub.lift_vector + sub.drag_vector;
        }
    }

    force_cache = total_force_local; last_local_wind = local_wind_node; last_transform = current_transform; dirty = false;
    return current_transform.basis.xform(total_force_local);
}

void AeroSurface::_process(double delta) {
    if (Engine::get_singleton()->is_editor_hint() || is_inside_tree()) {
        (void)compute_force(Variant(delta)); _update_debug_draw();
    }
}

TypedArray<Dictionary> AeroSurface::get_vortices() const {
    TypedArray<Dictionary> result;
    for (int i = 0; i < vortices.size(); i++) {
        const Vortex &v = vortices[i];
        Dictionary d;
        d["left_tip"] = v.left_tip; d["right_tip"] = v.right_tip; d["collocation_point"] = v.collocation_point; d["normal"] = v.normal; d["circulation"] = v.circulation; d["lift_vector"] = v.lift_vector; d["lift"] = v.lift; d["cl"] = v.cl; d["cl_vlm"] = v.cl_vlm; d["span"] = v.span; d["alpha"] = v.alpha;
        result.push_back(d);
    }
    return result;
}

TypedArray<Dictionary> AeroSurface::get_subsections() const {
    TypedArray<Dictionary> result;
    for (const auto& s : subsections) {
        Dictionary d; d["area"] = s.area; d["chord"] = s.chord; d["transform"] = s.transform; d["alpha"] = s.alpha;
        result.push_back(d);
    }
    return result;
}
