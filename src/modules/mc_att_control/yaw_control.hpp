#ifndef YAW_CONTROL_HPP
#define YAW_CONTROL_HPP

#include <cmath> // for cos() and sin()

struct YawControlInputs {
    float yawTarget; // Target yaw angle
};

struct YawControlState {
    float att[3];    // Array containing roll (att[0]), pitch (att[1]), yaw (att[2])
    float omegaf[3]; // Array containing angular velocity components
};

struct YawControlParameters {
    float YRC_Kp_psi; // Proportional gain for yaw rate control
};

class YawControl {
public:
    /**
     * Calculate the yaw rate command (r_cmd).
     * @param inputs Input structure containing the yaw target.
     * @param state Current state of the system (attitude and angular velocities).
     * @param params Parameters for yaw rate control.
     * @return Computed yaw rate command (r_cmd).
     */
    static float computeYawRateCmd(const YawControlInputs &inputs,
                                       const YawControlState &state,
                                       const YawControlParameters &params);
};

#endif // YAW_CONTROL_HPP;
