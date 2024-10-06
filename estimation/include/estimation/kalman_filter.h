#ifndef ESTIMATION_KALMAN_FILTER_H
#define ESTIMATION_KALMAN_FILTER_H

#include <Eigen/Dense>


//Template for this class setup:
// https://www.learncpp.com/cpp-tutorial/classes-and-header-files/?utm_content=cmp-true

//Kalman filter excample:
//https://github.com/hmartiro/kalman-cpp/blob/master/kalman.hpp

namespace estimation {
class kalman_filter
{
private:
    // A system matrix
    Eigen::MatrixXd aMatrix;
    //estimate error covariance matrix
    Eigen::MatrixXd pMatrix;

    //
    Eigen::MatrixXd& cMatrix; // output matrix

    // System dimensions
    int m, n;

    // Initial and current time
    double t0, t;

    // Discrete time step
    double dt;

    // Estimated states
    Eigen::VectorXd x_hat, x_hat_new;



public:
    //Constructor
    kalman_filter(Eigen::MatrixXd aMatrix,Eigen::VectorXd initialStateMatrix, Eigen::VectorXd initialPVector);
    //kalman_filter();

    /**
  * Update the estimated state based on measured values. The
  * time step is assumed to remain constant.
  */
    void update(const Eigen::VectorXd& y);

    /**
    * Update the estimated state based on measured values,
    * using the given time step and dynamics matrix.
    */
    void update(const Eigen::VectorXd& y, double dt, const Eigen::MatrixXd A);

};
}

#endif
