#include "URestimator.hpp"
#include <uORB/topics/vehicle_n.h>

URestimator::URestimator() : _attitude(Eigen::Vector3d::Zero()), _primary_axis(Eigen::Vector3d::Zero()) {}

URestimator::~URestimator() {
    if (_n_pub) {
        orb_unadvertise(_n_pub);
    }
}

void URestimator::update(const Eigen::Vector3d &att) {
    _attitude = att;

    // Compute the rotation matrix R_IB
    Eigen::Matrix3d R_IB = calculateRotationMatrix(_attitude);

    // Calculate primary axis in the inertial frame
    _primary_axis = (R_IB.transpose() * Eigen::Vector3d(0, 0, -1)).transpose();
}

const Eigen::Vector3d &URestimator::getPrimaryAxis() const {
    return _primary_axis;
}

void URestimator::publishPrimaryAxis(double timestamp) {
    struct vehicle_n_s n_msg{};
    n_msg.timestamp = timestamp;

    // Populate the n vector
    n_msg.n[0] = _primary_axis(0);
    n_msg.n[1] = _primary_axis(1);
    n_msg.n[2] = _primary_axis(2);

    if (_n_pub == nullptr) {
        _n_pub = orb_advertise(ORB_ID(vehicle_n), &n_msg);
    } else {
        orb_publish(ORB_ID(vehicle_n), _n_pub, &n_msg);
    }
}

Eigen::Matrix3d URestimator::calculateRotationMatrix(const Eigen::Vector3d &att) {
    double phi = att(0);   // Roll
    double theta = att(1); // Pitch
    double psi = att(2);   // Yaw

    Eigen::Matrix3d R_IB;
    R_IB << std::cos(theta) * std::cos(psi), std::cos(theta) * std::sin(psi), -std::sin(theta),
            std::sin(phi) * std::sin(theta) * std::cos(psi) - std::cos(phi) * std::sin(psi),
            std::sin(phi) * std::sin(theta) * std::sin(psi) + std::cos(phi) * std::cos(psi),
            std::sin(phi) * std::cos(theta),
            std::cos(phi) * std::sin(theta) * std::cos(psi) + std::sin(phi) * std::sin(psi),
            std::cos(phi) * std::sin(theta) * std::sin(psi) - std::sin(phi) * std::cos(psi),
            std::cos(phi) * std::cos(theta);

    return R_IB;
}
