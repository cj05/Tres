#pragma once

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/string.hpp>

namespace godot {

class XFoilParser : public Object {
    GDCLASS(XFoilParser, Object);

protected:
    static void _bind_methods();

public:
    static Dictionary get_clean_points(const String &p_raw_text);
};

} // namespace godot
