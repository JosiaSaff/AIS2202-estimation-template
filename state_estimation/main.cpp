#include "estimation/kalman_filter.h"

#include <rapidcsv.h>

int main()
{


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
