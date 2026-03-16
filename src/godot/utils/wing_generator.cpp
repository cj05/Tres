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
    double current_twist = 0.0;
    for (int i = 0; i < sections.size(); i++) {
        Ref<WingSection> s = sections[i];
        if (s.is_null()) continue;

        // Apply dihedral and twist to the basis
        // Dihedral is rotation around local X
        current_tf.basis = current_tf.basis.rotated(current_tf.basis.get_column(0).normalized(), Math::deg_to_rad(s->get_dihedral_angle()));
        // Twist is rotation around local Z
        current_tf.basis = current_tf.basis.rotated(current_tf.basis.get_column(2).normalized(), Math::deg_to_rad(s->get_twist()));
        
        // Offset origin in the newly oriented basis (protrudes forward along twisted chord)
        current_tf.origin += current_tf.basis.xform(Vector3(s->get_sweep_offset(), 0, s->get_span_offset()));

        current_twist += s->get_twist();

        WingStation st;
        st.transform = current_tf;
        st.chord = s->get_chord();
        st.airfoil = s->get_airfoil_data();
        st.twist = current_twist;
        stations.push_back(st);
    }
    return stations;
}

Vector<WingSubsection> WingGenerator::generate_subsections(const Vector<WingStation>& stations, double segments_per_meter, bool use_cosine_spacing) {
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

            if (use_cosine_spacing) {
                t1 = (1.0 - Math::cos(t1 * Math_PI)) * 0.5;
                t2 = (1.0 - Math::cos(t2 * Math_PI)) * 0.5;
            }

            double mid = (t1 + t2) * 0.5;

            WingSubsection sub;
            sub.transform.origin = s1.transform.origin.lerp(s2.transform.origin, mid);
            // Slerp will now handle the smooth transition of twist since it's in the basis
            sub.transform.basis = s1.transform.basis.slerp(s2.transform.basis, mid);

            sub.chord = Math::lerp(s1.chord, s2.chord, mid);
            sub.airfoil = (mid < 0.5) ? s1.airfoil : s2.airfoil;
            sub.span = segment_span * (t2 - t1);
            sub.area = sub.span * sub.chord;

            // To handle twist correctly in corners, we calculate basis at t1 and t2
            auto get_tf = [&](double t) {
                Transform3D tf;
                tf.origin = s1.transform.origin.lerp(s2.transform.origin, t);
                tf.basis = s1.transform.basis.slerp(s2.transform.basis, t);
                return tf;
            };

            Transform3D tf_left = get_tf(t1);
            Transform3D tf_right = get_tf(t2);
            double chord_left = Math::lerp(s1.chord, s2.chord, t1);
            double chord_right = Math::lerp(s1.chord, s2.chord, t2);

            sub.v1_4_left = tf_left.xform(Vector3(0.25 * chord_left, 0, 0));
            sub.v1_4_right = tf_right.xform(Vector3(0.25 * chord_right, 0, 0));
            sub.v3_4_left = tf_left.xform(Vector3(0.75 * chord_left, 0, 0));
            sub.v3_4_right = tf_right.xform(Vector3(0.75 * chord_right, 0, 0));

            subsections.push_back(sub);
        }
    }
    return subsections;
}

} // namespace godot
