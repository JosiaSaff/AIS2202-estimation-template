#ifndef ESTIMATION_KALMAN_FILTER_H
#define ESTIMATION_KALMAN_FILTER_H

#include <Eigen/Dense>


//Template for this class setup:
// https://www.learncpp.com/cpp-tutorial/classes-and-header-files/?utm_content=cmp-true

namespace estimation {
class kalman_filter
{
private:
    // A system matrix
    Eigen::MatrixXd aMatrix = aMatrix;
    //covariance matrix
    Eigen::MatrixXd pMatrix;
public:
    //Constructor
    kalman_filter(Eigen::MatrixXd aMatrix,Eigen::MatrixXd initialStateMatrix);
    kalman_filter();
};
}

#endif
