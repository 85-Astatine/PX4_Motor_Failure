#pragma once

#include <array>

struct Inputs {
    double xTarget;
    double yTarget;
    double zTarget;
};

struct State {
    std::array<double, 3> posf;
    std::array<double, 3> velf;
    std::array<double, 3> vel_ref;
    std::array<double, 3> pos_ref;
};

struct Params {
    double position_maxAngle;
    double position_maxVel;
    double position_Kp_pos;
    double position_Kp_vel;
    double position_Ki_vel;
    double position_intLim;
    double freq;
    double g;
};

class URPosControl {
public:
    URPosControl() = default;

    std::array<double, 3> control(const Inputs &inputs, State &state, const Params &par);

private:
    std::array<double, 3> errorInt = {0.0, 0.0, 0.0};
};
