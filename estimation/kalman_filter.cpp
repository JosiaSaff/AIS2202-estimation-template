#include "estimation/kalman_filter.h"

using namespace estimation;


// Implement kalman filter turtorial


kalman_filter::kalman_filter(){

}
//ispiration source: https://github.com/hmartiro/kalman-cpp/blob/master/kalman.cpp

void kalman_filter::update(const Eigen::VectorXd& y) {

    //if(!initialized)
    //    throw std::runtime_error("Filter is not initialized!");

    x_hat_new = aMatrix * x_hat;
    pMatrix = aMatrix * pMatrix* aMatrix.transpose() + Q;
    K = P*C.transpose()*(C*P*C.transpose() + R).inverse();
    x_hat_new += K * (y - C*x_hat_new);
    P = (I - K*C)*P;
    x_hat = x_hat_new;

    t += dt;
}

void kalman_filter::update(const Eigen::VectorXd& y, double dt, const Eigen::MatrixXd A) {

    this->A = A;
    this->dt = dt;
    update(y);
}


//The kalman filter addresses the general problem of trying to estimate the

// x_k = Ax_{k-1}+Bu_{k-1}+w_{k-1]
// z_k = Hx_k

//noicy measurement z = x + er
//with state matrix x = [[][]]

// time evolution:
// x_{k+1} = x_k

//we need to define how the system is evolving through time