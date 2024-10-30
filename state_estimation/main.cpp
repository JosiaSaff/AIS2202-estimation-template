#include "estimation/kalman_filter.h"

#include <rapidcsv.h>

int main()
{
    /*
     * Number of rows: 25
Number of columns per row (max): 100
The mass estimate is: 0.932043
The imu bias is:-0.00366194  0.00884945   0.0771078
The force bias is:  9.07633 -1.01814  9.98482
The calculated mass center is:     -0.00028 -5.44085e-05   -0.0439107, with the mass estimate of :0.932043.
The calculated  torque bias is:  0.432449 -0.692162 -0.156746
     */


//All of these can be found in paper 3
double mass = 0.93;
//massCenter
Eigen::Vector3d massCenter(0.0, 0.0, 0.044);

//Sensor Bias
Eigen::Vector3d ftsBias(9.07633, -1.01814, 9.98482);

//IMU Bias
Eigen::Vector3d imuBias(-0.00366194, 0.00884945, 0.0771078);


estimation::kalman_filter kalman(mass, massCenter, ftsBias, imuBias);

Eigen::Matrix3d rotationMatrix = Eigen::Matrix3d::Identity();
kalman.set_rotation_matrix(rotationMatrix);

//dw
//stedy state acceleration here??
Eigen::Vector3d imuAccel(0.0, 0.0, 9.81);

Eigen::Vector3d ftsForce(1.0, 1.0, 1.0);


Eigen::Vector3d ftsTorque(0.1, 0.1, 0.1);

kalman.predict();
kalman.update(imuAccel, ftsForce, ftsTorque);


Eigen::VectorXd contactWrench = kalman.get_contact_wrench();

std::cout << "Contact Wrench: \n" << contactWrench << std::endl;

return 0;

}
