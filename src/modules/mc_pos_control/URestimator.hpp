#ifndef URESTIMATOR_HPP
#define URESTIMATOR_HPP

#include <Eigen/Dense>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_n.h>

class URestimator {
public:
    URestimator();
    ~URestimator();

    void update(const Eigen::Vector3d &att);
    const Eigen::Vector3d &getPrimaryAxis() const;

    void publishPrimaryAxis(double timestamp);

private:
    Eigen::Vector3d _attitude;     // Attitude: [phi, theta, psi]
    Eigen::Vector3d _primary_axis; // Calculated primary axis (n)

    orb_advert_t _n_pub{nullptr}; // uORB publisher handle for vehicle_n

    Eigen::Matrix3d calculateRotationMatrix(const Eigen::Vector3d &att);
};

#endif // URESTIMATOR_HPP
