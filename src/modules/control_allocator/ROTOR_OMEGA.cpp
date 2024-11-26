#include "ROTOR_OMEGA.hpp"
#include <cmath>


RotorOmega::RotorOmega(){};

// Set coefficients
void RotorOmega::setCoefficients(double thrust_coeff, double torque_coeff) {
    thrust_coefficient = thrust_coeff;
    torque_coefficient = torque_coeff;
}

// Compute thrust based on angular velocity
double RotorOmega::computeThrust(double omega) const {
    return thrust_coefficient * std::pow(omega, 2);
}

// Compute torque based on angular velocity
double RotorOmega::computeTorque(double omega) const {
    return torque_coefficient * std::pow(omega, 2);
}

// Update rotor state using angular velocity vector and attitude
Eigen::Vector3d RotorOmega::updateRotorState(const Eigen::Vector3d &omega_vector, const Eigen::Vector3d &att) {
    double phi = att(0);
    double theta = att(1);
    double psi = att(2);

    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(psi, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(phi, Eigen::Vector3d::UnitX());

    Eigen::Vector3d rotor_forces;
    rotor_forces << computeThrust(omega_vector(0)), computeThrust(omega_vector(1)), computeThrust(omega_vector(2));

    Eigen::Vector3d world_forces = R * rotor_forces;

    return world_forces;
}

// Print thrust and torque
void RotorOmega::printRotorValues(double omega) const {
    double thrust = computeThrust(omega);
    double torque = computeTorque(omega);
    std::cout << "Thrust: " << thrust << ", Torque: " << torque << std::endl;
}


