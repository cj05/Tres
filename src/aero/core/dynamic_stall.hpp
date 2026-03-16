#pragma once
#include "aero_math.hpp"
#include <algorithm>
#include <cmath>


namespace aero {

struct StallInput {
    Real alpha;       // radians
    Real alpha_dot;   // rad/s
    Real velocity;    // m/s
    Real chord;       // m
    Real cl_vlm;      // prestall lift from VLM
    Real dt;          // timestep
};

struct StallOutput {
    Real cl_corrected;
};

class DynamicStallModel {
public:
    virtual ~DynamicStallModel() {}
    virtual StallOutput update(const StallInput& input) = 0;
    virtual void reset() = 0;
};


// VLM + Flat Plate Dynamic Stall Model
class CirculationLagModel : public DynamicStallModel {

    Real stall_state = 0.0;
    bool initialized = false;

    // tunable parameters
    Real alpha_stall = deg2rad(15.0);
    Real stall_width = deg2rad(5.0);
    Real k_tau = 2.0;          // lag time constant scale
    Real pitch_boost = 0.5;    // pitch rate lift gain

public:

    CirculationLagModel() {}

    CirculationLagModel(Real p_k_tau)
        : k_tau(p_k_tau) {}

    virtual StallOutput update(const StallInput& input) override {

        if (input.velocity < 0.1 || input.dt <= 0.0) {
            return { input.cl_vlm };
        }

        if (!initialized) {
            stall_state = 0.0;
            initialized = true;
        }

        // --------------------------
        // 1. Attached-flow baseline
        // --------------------------

        Real cl_attached = input.cl_vlm;

        // pitch-rate lift augmentation
        cl_attached += pitch_boost * input.alpha_dot * input.chord / input.velocity;


        // --------------------------
        // 2. Flat plate baseline
        // --------------------------

        Real cl_flat = std::sin(2.0 * input.alpha);


        // --------------------------
        // 3. Static stall estimate
        // --------------------------

        Real abs_alpha = std::abs(input.alpha);

        Real stall_static =
            (abs_alpha - alpha_stall) / stall_width;

        stall_static = std::clamp(stall_static, (Real)0.0, (Real)1.0);


        // --------------------------
        // 4. Dynamic stall lag
        // --------------------------

        Real tau = k_tau * input.chord / input.velocity;

        Real a = std::min(input.dt / tau, (Real)1.0);

        stall_state += (stall_static - stall_state) * a;


        // --------------------------
        // 5. Blend models
        // --------------------------

        Real cl =
            (1.0 - stall_state) * cl_attached +
            stall_state * cl_flat;

        return { cl };
    }

    virtual void reset() override {
        stall_state = 0.0;
        initialized = false;
    }
};

} // namespace aero