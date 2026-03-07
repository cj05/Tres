#pragma once

struct AeroInput {
    float rho;      // air density
    float speed;    // airspeed magnitude
    float alpha;    // angle of attack (rad)
    float area;     // reference area
    float chord;    // reference chord
};

struct AeroOutput {
    float lift;
    float drag;
    float moment;
};
