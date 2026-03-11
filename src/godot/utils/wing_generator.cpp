#include "wing_generator.hpp"

namespace godot {

Vector<WingStation> WingGenerator::generate_stations(const Ref<AeroGeometryProperties>& props) {
    Vector<WingStation> stations;
    if (props.is_null()) return stations;

    WingStation root;
    root.transform = Transform3D();
    root.chord = props->get_root_chord();
    root.airfoil = props->get_root_airfoil();
    root.twist = 0.0;
    stations.push_back(root);

    TypedArray<WingSection> sections = props->get_sections();
    Transform3D current_tf;
    for (int i = 0; i < sections.size(); i++) {
        Ref<WingSection> s = sections[i];
        if (s.is_null()) continue;

        current_tf.basis = current_tf.basis.rotated(Vector3(1, 0, 0), Math::deg_to_rad(s->get_dihedral_angle()));
        current_tf.origin += current_tf.basis.xform(Vector3(s->get_sweep_offset(), 0, s->get_span_offset()));

        WingStation st;
        st.transform = current_tf;
        st.chord = s->get_chord();
        st.airfoil = s->get_airfoil_data();
        st.twist = s->get_twist();
        stations.push_back(st);
    }
    return stations;
}

Vector<WingSubsection> WingGenerator::generate_subsections(const Vector<WingStation>& stations, double segments_per_meter) {
    Vector<WingSubsection> subsections;
    if (stations.size() < 2) return subsections;

    for (int i = 0; i < stations.size() - 1; i++) {
        const WingStation& s1 = stations[i];
        const WingStation& s2 = stations[i+1];

        double segment_span = s1.transform.origin.distance_to(s2.transform.origin);
        int num_sub = (int)Math::round(segment_span * segments_per_meter);
        if (num_sub < 1) num_sub = 1;

        for (int j = 0; j < num_sub; j++) {
            double t1 = (double)j / num_sub;
            double t2 = (double)(j + 1) / num_sub;
            double mid = (t1 + t2) * 0.5;

            WingSubsection sub;
            sub.transform.origin = s1.transform.origin.lerp(s2.transform.origin, mid);
            sub.transform.basis = s1.transform.basis.slerp(s2.transform.basis, mid);
            // Apply twist interpolation if needed
            double twist = Math::lerp(s1.twist, s2.twist, mid);
            if (Math::abs(twist) > 1e-6) {
                sub.transform.basis = sub.transform.basis.rotated(sub.transform.basis.xform(Vector3(0, 0, 1)), Math::deg_to_rad(twist));
            }

            sub.chord = Math::lerp(s1.chord, s2.chord, mid);
            sub.airfoil = (mid < 0.5) ? s1.airfoil : s2.airfoil;
            sub.span = segment_span * (t2 - t1);
            sub.area = sub.span * sub.chord;
            subsections.push_back(sub);
        }
    }
    return subsections;
}

} // namespace godot
