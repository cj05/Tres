#pragma once

#include <godot_cpp/classes/node.hpp>

namespace godot {

class AeroSurface;

class VLMTester : public Node {
    GDCLASS(VLMTester, Node);

    AeroSurface *aero_surface = nullptr;
    String last_result = "No tests run.";
    bool run_tests_trigger = false;
    bool verbose = true;

    protected:
    static void _bind_methods();

    public:
    VLMTester();
    ~VLMTester();

    void set_aero_surface(Node *p_surface);
    Node *get_aero_surface() const;

    void set_run_tests(bool p_run);
    bool get_run_tests() const { return false; }

    void set_last_result(String p_res) { last_result = p_res; }
    String get_last_result() const { return last_result; }

    void set_verbose(bool p_verbose) { verbose = p_verbose; }
    bool is_verbose() const { return verbose; }

    void _ready() override;

    void run_all_tests();
    void run_vlm_validation();
    void run_coherence_test();
    void run_cosine_spacing_test();
    void run_range_stability_test();
    void run_aoa_sweep_test();
    void run_detailed_debug();
    
private:
    void _report(const String& msg, bool success = true);
};

} // namespace godot
