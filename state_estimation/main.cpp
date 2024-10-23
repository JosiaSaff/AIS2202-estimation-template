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
Eigen::Vector3d imuBias(-0.00366194, 0.00884945, 0.0771078);  // Replace with actual IMU bias

// Instantiate the Kalman filter object
estimation::kalman_filter kf(mass, massCenter, ftsBias, imuBias);

// Set the rotation matrix (optional, as needed)
Eigen::Matrix3d rotationMatrix = Eigen::Matrix3d::Identity();  // Example
kf.set_rotation_matrix(rotationMatrix);


//stedy state acceleration here??
Eigen::Vector3d imuAccel(0.0, 0.0, 9.81);
Eigen::Vector3d ftsForce(1.0, 1.0, 1.0);
Eigen::Vector3d ftsTorque(0.1, 0.1, 0.1);

kf.predict();
kf.update(imuAccel, ftsForce, ftsTorque);


Eigen::VectorXd contactWrench = kf.get_contact_wrench();

std::cout << "Contact Wrench: \n" << contactWrench << std::endl;

return 0;

}
