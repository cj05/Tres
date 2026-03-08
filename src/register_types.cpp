#include "godot/aero_surface.hpp"
#include "godot/vlm_tester.hpp"
#include "godot/modular_component.hpp"
#include "godot/aero_component.hpp"
#include "godot/part.hpp"
#include "godot/tooled_properties.hpp"
#include "godot/geometry_component.hpp"
#include "godot/coefficient_curve.hpp"
#include "godot/airfoil_resource.hpp"
#include "godot/airfoil_util.hpp"
#include "godot/xfoil_parser.hpp"
#include "godot/curve_prediction.hpp"
#include "godot/wing_section.hpp"
#include "godot/aero_geometry_properties.hpp"
#include "godot/aero_geometry.hpp"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/godot.hpp>

using namespace godot;

void initialize_aero(ModuleInitializationLevel level) {
    if (level != MODULE_INITIALIZATION_LEVEL_SCENE) {
        return;
    }

    // Register Base Classes First
    GDREGISTER_CLASS(ModularComponent);
    GDREGISTER_CLASS(AeroComponent);
    GDREGISTER_CLASS(AeroSurface);
    GDREGISTER_CLASS(VLMTester);
    
    GDREGISTER_CLASS(Part);
    
    GDREGISTER_CLASS(TooledProperties);
    GDREGISTER_CLASS(AeroGeometryProperties);
    
    GDREGISTER_CLASS(GeometryComponent);
    GDREGISTER_CLASS(AeroGeometry);
    
    GDREGISTER_CLASS(CoefficientCurve);
    GDREGISTER_CLASS(AirfoilResource);
    
    // Utilities
    GDREGISTER_ABSTRACT_CLASS(AirfoilUtil);
    GDREGISTER_ABSTRACT_CLASS(XFoilParser);
    GDREGISTER_ABSTRACT_CLASS(CurvePrediction);
    
    GDREGISTER_CLASS(WingSection);
}

void uninitialize_aero(ModuleInitializationLevel level) {
    if (level != MODULE_INITIALIZATION_LEVEL_SCENE) {
        return;
    }
}

extern "C" {

GDExtensionBool GDE_EXPORT aeromodule_init(
        GDExtensionInterfaceGetProcAddress p_get_proc_address,
        GDExtensionClassLibraryPtr p_library,
        GDExtensionInitialization *r_initialization) {

    GDExtensionBinding::InitObject init_obj(p_get_proc_address, p_library, r_initialization);

    init_obj.register_initializer(initialize_aero);
    init_obj.register_terminator(uninitialize_aero);
    init_obj.set_minimum_library_initialization_level(MODULE_INITIALIZATION_LEVEL_SCENE);

    return init_obj.init();
}

}
