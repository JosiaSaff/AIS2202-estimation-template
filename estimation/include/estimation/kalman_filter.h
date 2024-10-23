#ifndef ESTIMATION_KALMAN_FILTER_H
#define ESTIMATION_KALMAN_FILTER_H

#include <Eigen/Dense>


//Template for this class setup:
// https://www.learncpp.com/cpp-tutorial/classes-and-header-files/?utm_content=cmp-true

//Kalman filter excample:
//https://github.com/hmartiro/kalman-cpp/blob/master/kalman.hpp

namespace estimation {
    class kalman_filter {
    private:

        Eigen::MatrixXd aMatrix, hMatrix, qMatrix, rMatrix, pMatrix, kMatrix;

        Eigen::VectorXd x;

        Eigen::MatrixXd Rws; // output matrix

        double mass;

        Eigen::Vector3d massCenter;

        Eigen::Vector3d ftsBias, imuBias;

    public:
        kalman_filter(double mass_, const Eigen::Vector3d& massCenter_, const Eigen::Vector3d& ftsBias_, const Eigen::Vector3d& imuBias_, int stateDim = 9, int measurementDim = 9)
                        : mass(mass_), massCenter(massCenter_), ftsBias(ftsBias_), imuBias(imuBias_) {

            // [ax, ay, az, fx, fy, fz, tx, ty, tz]
            x = Eigen::VectorXd::Zero(stateDim);

            aMatrix = Eigen::MatrixXd::Identity(stateDim, stateDim);
            hMatrix = Eigen::MatrixXd::Identity(measurementDim, stateDim);
            pMatrix = Eigen::MatrixXd::Identity(stateDim, stateDim);
            qMatrix = Eigen::MatrixXd::Identity(stateDim, stateDim) * 0.01; //tune
            rMatrix = Eigen::MatrixXd::Identity(measurementDim, measurementDim) * 0.1; //tune
            Rws = Eigen::Matrix3d::Identity();
        }

        void set_rotation_matrix(const Eigen::Matrix3d& R) {
            Rws = R;
        }

        void predict() {
            x = aMatrix * x;
            pMatrix = aMatrix * pMatrix * aMatrix.transpose() + qMatrix;
        }

        void update(const Eigen::Vector3d& imuAccel, const Eigen::Vector3d& ftsForce, const Eigen::Vector3d& ftsTorque) {
            Eigen::Vector3d imuCompensated = imuAccel - imuBias;
            Eigen::Vector3d forceCompensated = ftsForce - ftsBias;
            Eigen::Vector3d torqueCompensated = ftsTorque - (massCenter.cross(ftsBias));

            Eigen::Vector3d imuForce = mass * imuCompensated;
            Eigen::Vector3d imuTorque = massCenter.cross(imuForce);

            // Combine measurements
            Eigen::VectorXd z(9);
            z << imuCompensated, forceCompensated, torqueCompensated;

            // Kalman filter update step
            Eigen::VectorXd y = z - hMatrix * x;
            Eigen::MatrixXd S = hMatrix * pMatrix * hMatrix.transpose() + rMatrix;
            Eigen::MatrixXd K = pMatrix * hMatrix.transpose() * S.inverse();
            x = x + K * y;
            pMatrix = (Eigen::MatrixXd::Identity(9, 9) - K * hMatrix) * pMatrix;
        }

        Eigen::VectorXd get_contact_wrench() const {
            Eigen::VectorXd contactWrench = x.tail<6>();
            Eigen::Vector3d gravityForce = mass * Rws.transpose() * Eigen::Vector3d(0, 0, -9.81);
            contactWrench.head<3>() -= gravityForce;
            contactWrench.tail<3>() -= massCenter.cross(gravityForce);
            return contactWrench;
        }
    };
}
#endif
