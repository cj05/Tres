#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "aero/core/aero_math.hpp"

using namespace aero;

TEST_CASE("Aero Math: AoA Calculation") {
    SUBCASE("Zero AoA") {
        Vector3 v(10.0, 0.0, 0.0);
        CHECK(compute_aoa(v) == doctest::Approx(0.0));
    }

    SUBCASE("Positive AoA (Wind from below)") {
        // 45 degrees
        Vector3 v(10.0, 10.0, 0.0);
        CHECK(compute_aoa(v) == doctest::Approx(M_PI / 4.0));
    }

    SUBCASE("Negative AoA (Wind from above)") {
        // -45 degrees
        Vector3 v(10.0, -10.0, 0.0);
        CHECK(compute_aoa(v) == doctest::Approx(-M_PI / 4.0));
    }

    SUBCASE("Backwards flight (Wind from behind)") {
        // 180 degrees
        Vector3 v(-10.0, 0.0, 0.0);
        CHECK(std::abs(compute_aoa(v)) == doctest::Approx(M_PI));
    }

    SUBCASE("Zero velocity") {
        Vector3 v(0.0, 0.0, 0.0);
        CHECK(compute_aoa(v) == doctest::Approx(0.0));
    }
}
