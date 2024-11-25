#ifndef ALLOCATION_ATT_INDI_HPP
#define ALLOCATION_ATT_INDI_HPP

#include <Eigen/Dense>
#include <vector>
#include <cmath>

/**
 * Computes the vector h by applying the inverse of the rotation matrix R_IB to n_des.
 * @param n_des Desired vector in inertial coordinates (3D vector).
 * @param att Attitude angles (roll: att[0], pitch: att[1], yaw: att[2]) in radians.
 * @return Transformed vector h in body coordinates (3D vector).
 */
Eigen::Vector3d computeH0(const Eigen::Vector3d &n_des, const Eigen::Vector3d &att) {
	// Extract attitude angles
	double phi = att[0];   // Roll
	double theta = att[1]; // Pitch
	double psi = att[2];   // Yaw

	// Compute the rotation matrix R_IB
	Eigen::Matrix3d R_IB;
	R_IB << std::cos(psi) * std::cos(theta), std::cos(psi) * std::sin(theta) * std::sin(phi) - std::sin(psi) * std::cos(phi), std::cos(psi) * std::sin(theta) * std::cos(phi) + std::sin(psi) * std::sin(phi),
			std::sin(psi) * std::cos(theta), std::sin(psi) * std::sin(theta) * std::sin(phi) + std::cos(psi) * std::cos(phi), std::sin(psi) * std::sin(theta) * std::cos(phi) - std::cos(psi) * std::sin(phi),
			-std::sin(theta),                std::cos(theta) * std::sin(phi),                                                std::cos(theta) * std::cos(phi);

	// Compute the inverse of R_IB (R_IB is orthogonal, so R_IB^-1 = R_IB^T)
	Eigen::Matrix3d R_IB_T = R_IB.transpose();

	// Apply the transpose of R_IB to n_des to compute h = R_IB^T * n_des
	Eigen::Vector3d h = R_IB_T * n_des;

	return h;
}

/**
 * Computes the squared values of the input vector w and returns them as U.
 * @param w Input vector of angular velocities (4D vector).
 * @return A vector U containing the squared values of the input vector w.
 */
Eigen::Vector4d computeOmega(const Eigen::Vector4d &w) {
	Eigen::Vector4d U;

	// Compute the squared values of each element in w
	for (int i = 0; i < 4; ++i) {
		U[i] = w[i] * w[i];
	}

	return U;
}

/**
 * Computes the vertical acceleration zdd.
 * @param att Attitude angles (roll: att[0], pitch: att[1], yaw: att[2]) in radians.
 * @param a Acceleration vector in the body frame (3D vector).
 * @return The vertical acceleration zdd.
 */
double computeZdd(const Eigen::Vector3d &att, const Eigen::Vector3d &a) {
	// Extract Euler angles
	double phi = att[0];   // Roll
	double theta = att[1]; // Pitch
	double psi = att[2];   // Yaw

	// Compute the rotation matrix R from Euler angles
	Eigen::Matrix3d R;
	R << std::cos(psi) * std::cos(theta), std::cos(psi) * std::sin(theta) * std::sin(phi) - std::sin(psi) * std::cos(phi), std::cos(psi) * std::sin(theta) * std::cos(phi) + std::sin(psi) * std::sin(phi),
		 std::sin(psi) * std::cos(theta), std::sin(psi) * std::sin(theta) * std::sin(phi) + std::cos(psi) * std::cos(phi), std::sin(psi) * std::sin(theta) * std::cos(phi) - std::cos(psi) * std::sin(phi),
		 -std::sin(theta),                std::cos(theta) * std::sin(phi),                                                std::cos(theta) * std::cos(phi);

	// Transform the acceleration vector a using R
	Eigen::Vector3d f = R * a;

	// Compute zdd as the vertical acceleration
	double zdd = f[2] + 9.8124; // Add gravity compensation

	return zdd;
}


struct State {
	int fail_id = 0;                                  // Rotor failure identifier (0 means no failure)
	Eigen::Vector3d att = Eigen::Vector3d(0.0, 0.0, 0.0); // Attitude: [phi (roll), theta (pitch), psi (yaw)]
	// Additional state variables required by computeH0, computeOmega, and computeZdd
	Eigen::Vector3d n_des; // Desired inertial vector
	Eigen::Vector4d w;     // Angular velocities
	Eigen::Vector3d a;     // Body frame acceleration
	Eigen::Vector3d h0;    // Intermediate calculation vector
	Eigen::Vector4d U0;    // Initial control inputs
	double zdd;            // Desired vertical acceleration
};
struct ParamsIndi {
	double k0 = 1.0;               // Proportional gain
	double t0 = 0.1;               // Torque constant
	double b = 0.5;                // Arm length along one axis
	double l = 0.5;                // Arm length along the other axis
	double mass = 1.5;             // Mass of the system (kg)
	double Ix = 0.02;              // Moment of inertia about X-axis (kg·m²)
	double Iy = 0.02;              // Moment of inertia about Y-axis (kg·m²)
	double Iz = 0.04;              // Moment of inertia about Z-axis (kg·m²)
	double chi = 30.0;             // Configuration angle (degrees)
	bool DRF_enable = true;        // Damage recovery flag (true by default)
};
/**
 * @brief Control allocation function for INDI attitude control with fault handling.
 *
 * @param state      Current state of the system (includes fail_id, att, h0, zdd, U0, etc.)
 * @param nu         Desired dynamics (4D vector)
 * @param ddY        External disturbances (2D vector)
 * @param rdot       Current Yaw rate derivative (scalar)
 * @param par        System parameters (mass, inertia, configuration, etc.)
 * @param U          Output control inputs (4D vector)
 * @param Y          Output dynamics error (4D vector)
 * @param dU         Output control increment (4D vector)
 */
void allocation_att_indi(
	const State& state,
	const Eigen::Vector4d& nu,
	const Eigen::Vector2d& ddY,
	double rdot,
	const ParamsIndi& par,
	Eigen::Vector4d& U,
	Eigen::Vector4d& Y,
	Eigen::Vector4d& dU
) {
	int fail_flag = state.fail_id;
	double phi = state.att(0);     // Roll
	double theta = state.att(1);   // Pitch
	Eigen::Vector3d h0 = state.h0;
	double zdd = state.zdd;
	Eigen::Vector4d U0 = state.U0;

	double k = par.k0;
	double t = par.t0;
	double b = std::sqrt(par.b * par.b + par.l * par.l);
	double m = par.mass;
	double Ix = par.Ix;
	double Iy = par.Iy;
	double Iz = par.Iz;
	double beta = std::atan(par.b / par.l);
	double chi = par.chi / 57.3;

	if (fail_flag == 1 || fail_flag == 3) {
		chi = M_PI - par.chi / 57.3;
	}

	double h1 = h0(0);
	double h2 = h0(1);
	double h3 = h0(2);

	Eigen::Vector4d Gp, Gq, Gr;
	Gp = (Eigen::Vector4d() << k * b * std::sin(beta), -k * b * std::sin(beta),
		  -k * b * std::sin(beta), k * b * std::sin(beta)).finished() / Ix;

	Gq = (Eigen::Vector4d() << k * b * std::cos(beta), k * b * std::cos(beta),
		  -k * b * std::cos(beta), -k * b * std::cos(beta)).finished() / Iy;

	Gr = (Eigen::Vector4d() << t, -t, t, -t).finished() / Iz;

	Eigen::Matrix4d G0;
	G0.row(0) = -k / m * std::cos(theta) * std::cos(phi) * Eigen::Vector4d::Ones();
	G0.row(1) = -h3 * Gq + h2 * Gr;
	G0.row(2) = h3 * Gp - h1 * Gr;
	G0.row(3) = Gr;

	Eigen::Matrix4d R = Eigen::Matrix4d::Identity();
	R.block<2, 2>(1, 1) << std::cos(chi), std::sin(chi),
						   -std::sin(chi), std::cos(chi);

	Eigen::Vector4d ddy0;
	ddy0 << zdd, ddY(0), ddY(1), rdot;

	Eigen::Matrix4d G = R * G0;

	std::vector<int> fail_id;
	if (par.DRF_enable && fail_flag > 0) {
		fail_id = (fail_flag == 1 || fail_flag == 3) ? std::vector<int>{0, 2} : std::vector<int>{1, 3};
	} else if (fail_flag > 0) {
		fail_id = {fail_flag - 1};
	}

	if (fail_flag > 0) {
		for (int idx : fail_id) {
			G.col(idx).setZero();
		}

		if (par.DRF_enable) {
			ddy0.segment<2>(2).setZero();
			G.block<2, 4>(2, 0).setZero();
		} else {
			ddy0(3) = 0.0;
			G.row(3).setZero();
		}
	}
	
	dU = G.completeOrthogonalDecomposition().solve(nu - ddy0);
	Y = nu - ddy0;
	U = U0 + dU;

	if (fail_flag > 0) {
		for (int idx : fail_id) {
			U(idx) = 0.0;
		}
	}
}



#endif // ALLOCATION_ATT_INDI_HPP
