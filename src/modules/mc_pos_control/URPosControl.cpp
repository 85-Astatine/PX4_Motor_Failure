#include "URPosControl.hpp"
#include <cmath>
#include <px4_platform_common/log.h>
#include <algorithm>
#include <iostream>

std::array<double, 3> URPosControl::control(const Inputs &inputs, State &state, const Params &par) {
    double maxAngle = par.position_maxAngle;

    // Position control
    std::array<double, 3> errorPos = {
        inputs.xTarget - state.posf[0],
        inputs.yTarget - state.posf[1],
        inputs.zTarget - state.posf[2]
    };

    std::array<double, 3> velTarget = {
        par.position_Kp_pos * errorPos[0],
        par.position_Kp_pos * errorPos[1],
        par.position_Kp_pos * errorPos[2]
    };

    double maxVel = par.position_maxVel;
    for (int i = 0; i < 3; ++i) {
        velTarget[i] = std::max(std::min(velTarget[i], maxVel), -maxVel);
    }

    state.vel_ref = velTarget;
    state.pos_ref = {inputs.xTarget, inputs.yTarget, inputs.zTarget};

    // Velocity control
    std::array<double, 3> errorVel = {
        velTarget[0] - state.velf[0],
        velTarget[1] - state.velf[1],
        velTarget[2] - state.velf[2]
    };

    for (int i = 0; i < 3; ++i) {
        errorInt[i] += errorVel[i] / par.freq;
        errorInt[i] = std::max(std::min(errorInt[i], par.position_intLim), -par.position_intLim);
    }

    // Reference acceleration
    std::array<double, 3> a_ref = {
        par.position_Kp_vel * errorVel[0] + par.position_Ki_vel * errorInt[0],
        par.position_Kp_vel * errorVel[1] + par.position_Ki_vel * errorInt[1],
        par.position_Kp_vel * errorVel[2] + par.position_Ki_vel * errorInt[2] - par.g
    };

    double maxLateral = std::abs(par.g * std::tan(maxAngle));
    double latRatio = std::sqrt(a_ref[0] * a_ref[0] + a_ref[1] * a_ref[1]) / maxLateral;
    a_ref[0] /= std::max(latRatio, 1.0);
    a_ref[1] /= std::max(latRatio, 1.0);

    // Normalize the output
    double norm_a_ref = std::sqrt(a_ref[0] * a_ref[0] + a_ref[1] * a_ref[1] + a_ref[2] * a_ref[2]);
    std::array<double, 3> n_des = {a_ref[0] / norm_a_ref, a_ref[1] / norm_a_ref, a_ref[2] / norm_a_ref};
//     PX4_INFO("n_des: [%f, %f, %f]", n_des[0], n_des[1], n_des[2]);

    return n_des;

}
