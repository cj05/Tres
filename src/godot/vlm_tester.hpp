#pragma once

#include <godot_cpp/classes/node.hpp>

namespace godot {

class VLMTester : public Node {
    GDCLASS(VLMTester, Node);

protected:
    static void _bind_methods();

public:
    VLMTester();
    ~VLMTester();

    void _ready() override;
    void run_vlm_validation();
    void run_coherence_test();
    void run_cosine_spacing_test();
};

} // namespace godot
