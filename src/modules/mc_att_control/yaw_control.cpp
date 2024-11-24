#include "yaw_control.hpp"

float YawControl::computeYawRateCmd(const YawControlInputs &inputs,
                                        const YawControlState &state,
                                        const YawControlParameters &params) {
    // Calculate the yaw error
    float psiError = inputs.yawTarget - state.att[2];

    // Compute the desired yaw rate command
    float psi_dot_cmd = psiError * params.YRC_Kp_psi;

    // Compute r_cmd using the attitude and angular velocity
    float r_cmd = psi_dot_cmd * std::cos(state.att[0]) * std::cos(state.att[1])
                  - std::sin(state.att[0]) * state.omegaf[1];

    return r_cmd;
}
