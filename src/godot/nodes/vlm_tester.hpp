#pragma once

#include <godot_cpp/classes/node.hpp>

namespace godot {

class AeroSurface;

class VLMTester : public Node {
    GDCLASS(VLMTester, Node);

    AeroSurface *aero_surface = nullptr;

protected:
    static void _bind_methods();

public:
    VLMTester();
    ~VLMTester();

    void set_aero_surface(Node *p_surface);
    Node *get_aero_surface() const;

    void _ready() override;
    void run_vlm_validation();
    void run_coherence_test();
    void run_cosine_spacing_test();
    void run_aoa_sweep_test();
};

} // namespace godot
