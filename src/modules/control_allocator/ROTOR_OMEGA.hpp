#pragma once
#include <Eigen/Dense>
#include <iostream> // For printing values

class RotorOmega {
private:
    double thrust_coefficient;
    double torque_coefficient;

public:
    // Constructor
    RotorOmega();

    // Set to initialize coefficients
    void setCoefficients(double thrust_coeff, double torque_coeff);

    // Compute thrust
    double computeThrust(double omega) const;

    // Compute torque
    double computeTorque(double omega) const;

    // Update rotor state
    Eigen::Vector3d updateRotorState(const Eigen::Vector3d &omega_vector, const Eigen::Vector3d &att);

    // Print thrust and torque
    void printRotorValues(double omega) const;
};

