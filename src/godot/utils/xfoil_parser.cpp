#include "xfoil_parser.hpp"
#include <godot_cpp/variant/packed_string_array.hpp>
#include <godot_cpp/variant/packed_float32_array.hpp>

using namespace godot;

void XFoilParser::_bind_methods() {
    ClassDB::bind_static_method("XFoilParser", D_METHOD("get_clean_points", "raw_text"), &XFoilParser::get_clean_points);
}

Dictionary XFoilParser::get_clean_points(const String &p_raw_text) {
    PackedFloat32Array alpha;
    PackedFloat32Array cl;
    PackedFloat32Array cd;
    PackedFloat32Array cm;

    PackedStringArray lines = p_raw_text.split("\n");
    bool data_started = false;

    for (int i = 0; i < lines.size(); i++) {
        String line = lines[i].strip_edges();
        if (line.begins_with("------")) {
            data_started = true;
            continue;
        }

        if (data_started && line != "") {
            PackedStringArray parts = line.split(" ", false);
            if (parts.size() >= 5) {
                alpha.append(parts[0].to_float());
                cl.append(parts[1].to_float());
                cd.append(parts[2].to_float());
                cm.append(parts[4].to_float());
            }
        }
    }

    Dictionary results;
    results["alpha"] = alpha;
    results["cl"] = cl;
    results["cd"] = cd;
    results["cm"] = cm;
    return results;
}
