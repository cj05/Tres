#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct Vec3 {
    double x, y, z;
    Vec3(double x=0, double y=0, double z=0) : x(x), y(y), z(z) {}
    Vec3 operator+(const Vec3& v) const { return Vec3(x+v.x, y+v.y, z+v.z); }
    Vec3 operator-(const Vec3& v) const { return Vec3(x-v.x, y-v.y, z-v.z); }
    Vec3 operator*(double s) const { return Vec3(x*s, y*s, z*s); }
    Vec3 operator/(double s) const { return Vec3(x/s, y/s, z/s); }
    double dot(const Vec3& v) const { return x*v.x + y*v.y + z*v.z; }
    Vec3 cross(const Vec3& v) const { return Vec3(y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x); }
    double length() const { return std::sqrt(x*x + y*y + z*z); }
    double length_squared() const { return x*x + y*y + z*z; }
    Vec3 normalized() const { double l = length(); return l > 0 ? (*this) * (1.0/l) : Vec3(); }
};

struct Panel {
    Vec3 left_tip, right_tip, collocation, normal;
    double circulation;
};

// Mathematically stable Biot-Savart segment
Vec3 calculate_induced(const Vec3& P, const Vec3& V1, const Vec3& V2, double G) {
    Vec3 r1 = P - V1;
    Vec3 r2 = P - V2;
    Vec3 r1xr2 = r1.cross(r2);
    double r1xr2_mag2 = r1xr2.length_squared();
    const double rc2 = 1e-8; 
    double r1_m = r1.length();
    double r2_m = r2.length();
    if (r1_m < 1e-12 || r2_m < 1e-12) return Vec3();
    Vec3 r0 = V2 - V1;
    double term = (G / (4.0 * M_PI)) * (r0.dot(r1/r1_m - r2/r2_m)) / (r1xr2_mag2 + rc2);
    return r1xr2 * term;
}

Vec3 calculate_horseshoe(const Vec3& P, const Panel& src, double G, const Vec3& wind_dir) {
    Vec3 v_total(0,0,0);
    v_total = v_total + calculate_induced(P, src.left_tip, src.right_tip, G);
    auto semi_infinite = [&](const Vec3& p1, const Vec3& dir, double g) {
        Vec3 r = P - p1;
        Vec3 dxr = dir.cross(r);
        double dxr_m2 = dxr.length_squared();
        double r_m = r.length();
        if (r_m < 1e-12) return Vec3();
        const double rc2 = 1e-8;
        return dxr * ( (g / (4.0 * M_PI)) * (1.0 + dir.dot(r)/r_m) / (dxr_m2 + rc2) );
    };
    v_total = v_total + semi_infinite(src.left_tip, wind_dir, -G);
    v_total = v_total + semi_infinite(src.right_tip, wind_dir, G);
    return v_total;
}

void solve_vlm(std::vector<Panel>& panels, Vec3 wind) {
    int N = panels.size();
    std::vector<std::vector<double>> A(N, std::vector<double>(N));
    std::vector<double> b(N);
    Vec3 wind_dir = wind.normalized();
    for (int i=0; i<N; ++i) {
        b[i] = -wind.dot(panels[i].normal);
        for (int j=0; j<N; ++j) {
            A[i][j] = calculate_horseshoe(panels[i].collocation, panels[j], 1.0, wind_dir).dot(panels[i].normal);
        }
    }
    for (int i=0; i<N; i++) {
        double s = 1.0 / A[i][i];
        for (int j=0; j<N; j++) A[i][j] *= s;
        b[i] *= s;
    }
    for (int i=0; i<N; ++i) {
        int p = i;
        for (int j=i+1; j<N; ++j) if (std::abs(A[j][i]) > std::abs(A[p][i])) p = j;
        std::swap(A[i], A[p]); std::swap(b[i], b[p]);
        if (std::abs(A[i][i]) < 1e-25) continue;
        for (int j=i+1; j<N; ++j) {
            double f = A[j][i] / A[i][i];
            b[j] -= f * b[i];
            for (int k=i; k<N; ++k) A[j][k] -= f * A[i][k];
        }
    }
    std::vector<double> x(N);
    for (int i=N-1; i>=0; --i) {
        double s = 0;
        for (int j=i+1; j<N; ++j) s += A[i][j] * x[j];
        x[i] = (b[i] - s) / A[i][i];
    }
    for (int i=0; i<N; ++i) panels[i].circulation = x[i];
}

int main() {
    const int N = 10; // Number of panels
    const double span = 10.0;
    const double chord = 1.0;
    
    std::cout << "--- VLM Equal Spacing Table (Gamma vs AoA) ---" << std::endl;
    std::cout << "AoA | Circulation Distribution (Left -> Center -> Right)" << std::endl;
    std::cout << "--------------------------------------------------------" << std::endl;

    for (double aoa_deg = 0.0; aoa_deg <= 20.1; aoa_deg += 5.0) {
        double alpha_rad = aoa_deg * M_PI / 180.0;
        Vec3 wind(20.0 * std::cos(alpha_rad), 20.0 * std::sin(alpha_rad), 0);
        
        std::vector<Panel> panels(N);
        for (int i=0; i<N; ++i) {
            // Equal (Uniform) Spacing
            double y1 = -span/2.0 + (span/N) * i;
            double y2 = -span/2.0 + (span/N) * (i + 1);
            double y_mid = (y1 + y2) * 0.5;
            
            panels[i].left_tip = Vec3(0.25, 0, y1);
            panels[i].right_tip = Vec3(0.25, 0, y2);
            panels[i].collocation = Vec3(0.75, 0, y_mid);
            panels[i].normal = Vec3(0, 1, 0);
        }

        solve_vlm(panels, wind);

        std::cout << std::setw(3) << (int)aoa_deg << " | ";
        for (int i=0; i<N; ++i) {
            std::cout << std::setw(7) << std::fixed << std::setprecision(2) << panels[i].circulation << " ";
            if (i == N/2 - 1) std::cout << "| ";
        }
        std::cout << std::endl;
    }
    
    return 0;
}
