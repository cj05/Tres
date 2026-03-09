#pragma once

#ifdef STANDALONE_VLM
#include <vector>
#include <cmath>
#include <iostream>

namespace godot {
    struct Vector3 {
        double x, y, z;
        Vector3(double px=0, double py=0, double pz=0) : x(px), y(py), z(pz) {}
        Vector3 operator+(const Vector3& v) const { return Vector3(x+v.x, y+v.y, z+v.z); }
        Vector3 operator-(const Vector3& v) const { return Vector3(x-v.x, y-v.y, z-v.z); }
        Vector3 operator*(double s) const { return Vector3(x*s, y*s, z*s); }
        Vector3 operator-() const { return Vector3(-x, -y, -z); }
        void operator+=(const Vector3& v) { x+=v.x; y+=v.y; z+=v.z; }
        double dot(const Vector3& v) const { return x*v.x + y*v.y + z*v.z; }
        Vector3 cross(const Vector3& v) const { return Vector3(y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x); }
        double length() const { return std::sqrt(x*x + y*y + z*z); }
        double length_squared() const { return x*x + y*y + z*z; }
        Vector3 normalized() const { double l = length(); return l > 0 ? (*this) * (1.0/l) : Vector3(); }
    };

    template<typename T>
    struct Vector {
        std::vector<T> _data;
        void push_back(const T& val) { _data.push_back(val); }
        size_t size() const { return _data.size(); }
        T& operator[](size_t i) { return _data[i]; }
        const T& operator[](size_t i) const { return _data[i]; }
        void resize(size_t s) { _data.resize(s); }
        T* write_data() { return _data.data(); }
        // Minimal shim for Godot's Vector::write
        struct Write {
            T* ptr;
            T& operator[](size_t i) { return ptr[i]; }
        };
        Write get_write() { return Write{_data.data()}; }
        #define write get_write()
    };

    struct Math {
        static double abs(double v) { return std::abs(v); }
    };
    #define Math_PI 3.14159265358979323846
}
#else
#include <godot_cpp/variant/vector3.hpp>
#include <godot_cpp/templates/vector.hpp>
#endif

namespace godot {

struct VLMPanel {
    Vector3 left_tip;
    Vector3 right_tip;
    Vector3 collocation_point;
    Vector3 normal;
    double area;
    double chord;
    double circulation = 0.0;
};

class VLMSolver {
public:
    static void solve(Vector<VLMPanel> &p_panels, const Vector3 &p_wind_velocity);

private:
    static Vector3 _calculate_induced_velocity(const Vector3 &p_point, const Vector3 &p_v1, const Vector3 &p_v2, double p_gamma);
    static Vector3 _calculate_horseshoe_velocity(const Vector3 &p_point, const VLMPanel &p_panel, double p_gamma, const Vector3 &p_wind_dir);
};

} // namespace godot
