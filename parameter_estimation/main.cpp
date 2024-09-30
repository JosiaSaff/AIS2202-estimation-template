#include <rapidcsv.h>
#include <Eigen/Dense>



Eigen::MatrixXd pseudo_inverse(const Eigen::MatrixXd &G, const Eigen::MatrixXd &F)
{
    auto pseudoInv = (G.transpose() * G).completeOrthogonalDecomposition().pseudoInverse() * (G.transpose() * F);
    return pseudoInv;
}

double estimationOfMass(const Eigen::VectorXd &gravityVec, const Eigen::MatrixXd &forceVec){
    return ((gravityVec.transpose() * gravityVec).completeOrthogonalDecomposition().pseudoInverse() * (gravityVec.transpose() * forceVec)).value();
}


Eigen::VectorXd massCenterEstimation(const Eigen::MatrixXd &gravityVecg, const Eigen::MatrixXd &tourqeVec, double mStar){
    return (1/mStar)*(gravityVecg.transpose() * gravityVecg).inverse() * gravityVecg.transpose() * tourqeVec;
}

Eigen::MatrixXd gravityCompensation(const Eigen::VectorXd& gravityVec, const Eigen::MatrixXd& rotMatrix, double massEstimated, const Eigen::VectorXd &torqueVec) {
    Eigen::Vector3d gravityForce = massEstimated * (rotMatrix.transpose() * gravityVec);
    Eigen::Vector3d centerOfMass = massCenterEstimation(gravityVec, torqueVec, massEstimated);

    Eigen::Vector3d gravityTorque = centerOfMass.cross(gravityForce);
    Eigen::MatrixXd compensation(6, 1);
    compensation.block<3,1>(0,0) = gravityForce;
    compensation.block<3,1>(3,0) = gravityTorque;
    return compensation;
}


int main()
{
    return 0;
}
