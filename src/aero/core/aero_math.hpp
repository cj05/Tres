#pragma once

#include <cmath>
#include <vector>

namespace aero {

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using Real = double;

constexpr Real deg2rad(Real deg)
{
    return deg * M_PI / (Real)180.0;
}

constexpr Real rad2deg(Real rad)
{
    return rad * (Real)180.0 / M_PI;
}

struct Vector3 {
    Real x, y, z;

    Vector3(Real px = 0, Real py = 0, Real pz = 0) : x(px), y(py), z(pz) {}

    Vector3 operator+(const Vector3& v) const { return Vector3(x + v.x, y + v.y, z + v.z); }
    Vector3 operator-(const Vector3& v) const { return Vector3(x - v.x, y - v.y, z - v.z); }
    Vector3 operator*(Real s) const { return Vector3(x * s, y * s, z * s); }
    Vector3 operator/(Real s) const { return Vector3(x / s, y / s, z / s); }
    Vector3 operator-() const { return Vector3(-x, -y, -z); }

    void operator+=(const Vector3& v) { x += v.x; y += v.y; z += v.z; }
    void operator-=(const Vector3& v) { x -= v.x; y -= v.y; z -= v.z; }
    void operator*=(Real s) { x *= s; y *= s; z *= s; }

    Real dot(const Vector3& v) const { return x * v.x + y * v.y + z * v.z; }
    Vector3 cross(const Vector3& v) const {
        return Vector3(
            y * v.z - z * v.y,
            z * v.x - x * v.z,
            x * v.y - y * v.x
        );
    }

    Real length_squared() const { return x * x + y * y + z * z; }
    Real length() const { return std::sqrt(length_squared()); }

    Vector3 normalized() const {
        Real l = length();
        return (l > 1e-12) ? (*this) * (1.0 / l) : Vector3();
    }

};

template<typename T>
using Vector = std::vector<T>;

inline double compute_aoa(const Vector3 &v)
{
    // Standard Aerodynamic AoA: atan2(V_up, V_forward)
    // Here we assume X is the chord axis (forward = +X) and Y is the lift axis (up = +Y).
    // Note: If v is wind velocity, +X means wind from front, +Y means wind from below.
    if (std::abs(v.x) < 1e-9 && std::abs(v.y) < 1e-9) return 0.0;
    return std::atan2(v.y, v.x);
}

} // namespace aero
