#pragma once

#include "aero/core/aero_model.hpp"

namespace aero {

class FlatPlateModel : public IAeroModel {
public:
    AeroOutput compute(const AeroInput& input) override;
};

} // namespace aero
