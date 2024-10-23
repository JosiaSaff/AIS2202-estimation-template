#include <rapidcsv.h>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <sstream>
#include <filesystem>
#include <math.h>
#include <vector>
#include "../estimation/include/estimation/kalman_filter.h"

// https://www.ibm.com/docs/en/zos/2.4.0?topic=only-explicit-initialization-constructors-c

//Make sure parameter_estimation folder is used


Eigen::MatrixXd pseudo_inverse(const Eigen::MatrixXd &G, const Eigen::MatrixXd &F)
{
    auto pseudoInv = (G.transpose() * G).completeOrthogonalDecomposition().pseudoInverse() * (G.transpose() * F);
    return pseudoInv;
}

double estimationOfMass(Eigen::MatrixXd &gravityVecs, Eigen::MatrixXd &forceVecs){
    return ((gravityVecs.transpose() * gravityVecs).completeOrthogonalDecomposition().pseudoInverse() * (gravityVecs.transpose() * forceVecs)).value();
}


Eigen::Vector3d calculateTorqueBias(const Eigen::MatrixXd &torqueMeasurements, int startXIndex, int count) {
    Eigen::Vector3d torqueBias;
//    std::cout << "First row :  " << torqueMeasurements.col(0).segment(startXIndex, count) << std::endl;
//    std::cout << "torque measurements:  " << torqueMeasurements << std::endl;
    torqueBias(0) = torqueMeasurements.col(0).segment(startXIndex+1, count).mean();
    torqueBias(1) = torqueMeasurements.col(1).segment(startXIndex+1+count, count).mean();
    torqueBias(2) = torqueMeasurements.col(2).segment(startXIndex+1+(2*count), count).mean();

    return torqueBias;
}

    Eigen::Vector3d calculateForceBias(const Eigen::MatrixXd &forceMeasurements, int rows){
        Eigen::VectorXd forceBias = Eigen::VectorXd::Zero(3);
        forceBias(0) = forceMeasurements(0)/rows;
        forceBias(1) = forceMeasurements(1)/rows;
        forceBias(2) = forceMeasurements(2)/rows;
        return forceBias;
    }

Eigen::MatrixXd estimateMassCenter(const Eigen::MatrixXd &gravityVecs, const Eigen::MatrixXd &torqueVecs, double massEstimate, int row) {
    Eigen::MatrixXd aTorque = Eigen::MatrixXd::Zero(3 * row, 6);
    Eigen::VectorXd bTorque = Eigen::VectorXd::Zero(3 * row);

    for (int i = 1; i < row; ++i) {
        Eigen::Matrix3d gravityMatrix;
        gravityMatrix <<  0, -gravityVecs(i, 2), gravityVecs(i, 1),
                gravityVecs(i, 2), 0, -gravityVecs(i, 0),
                -gravityVecs(i, 1), gravityVecs(i, 0), 0;

        aTorque.block<3, 3>(3*i, 3) = massEstimate * gravityMatrix;

        bTorque(3*i) = torqueVecs(i, 0);
        bTorque(3*i + 1) = torqueVecs(i, 1);
        bTorque(3*i + 2) = torqueVecs(i, 2);
    }

    Eigen::VectorXd xTorque = aTorque.colPivHouseholderQr().solve(bTorque);
    Eigen::Vector3d massCenter = xTorque.tail(3);
    return massCenter;
}


//There are 24 measurements, in a total of 25 lines in the calibration_fts dataset


    Eigen::Vector3d calculateIMUBias(const Eigen::VectorXd &imuMeasurements, int row) {
        //Eigen::VectorXd imuBias = Eigen::VectorXd::Zero(3);
        Eigen::VectorXd imuBias(3);
        //imuBias(2) = imuMeasurements.col(2).mean();

        for(int i = 0; i < 3; i++)
        imuBias(i) =  imuMeasurements(i)/static_cast<float>(row);

        return imuBias;
    }


    const int MAX_ROWS = 100;
    const int MAX_COLS = 100;


    const std::string filePath = "../../datasets/0-calibration_fts-accel.csv";

    int main() {
        if (!std::filesystem::exists(filePath)) {
            std::cout << filePath << std::endl;
            std::cout << "FILE DO NOT EXIST!" << std::endl;
            return 1;
        }

        std::ifstream file(filePath);


        if (!file.is_open()) {
            std::cout << "Error, file is probably already open" << std::endl;
            return 1;
        }

        std::string data[MAX_ROWS][MAX_COLS];
        std::string line;
        int row = 0;

        while (getline(file, line) && row < MAX_ROWS) {
            std::stringstream ss(line);
            std::string cell;
            int col = 0;

            while (getline(ss, cell, ',') && col < MAX_COLS) {
                data[row][col] = cell;
                col++;
            }
            row++;
        }
        file.close();

        for (int i = 0; i < row; ++i) {
            for (int j = 0; j < MAX_COLS; ++j) {
//                std::cout << data[i][j] << " ";
            }
//            std::cout << std::endl;
        }

        std::cout << "Number of rows: " << row << std::endl;
        std::cout << "Number of columns per row (max): " << MAX_COLS << std::endl;


        Eigen::MatrixXd gravityVecs = Eigen::MatrixXd::Zero(row, 3);
        Eigen::MatrixXd forceVecs = Eigen::MatrixXd::Zero(row, 3);
        Eigen::VectorXd AxyzStacked =  Eigen::VectorXd::Zero(3);
        Eigen::VectorXd measurementsOfForce(3);  // Resize vector to have 'row' elements
        Eigen::MatrixXd torqueMeasurements = Eigen::MatrixXd::Zero(row, 3);
        Eigen::MatrixXd gravityMatrix = Eigen::MatrixXd::Zero(row, 3);
        Eigen::VectorXd stackedTorques = Eigen::VectorXd::Zero(3);
        Eigen::MatrixXd stackedGravMatrices = Eigen::MatrixXd::Zero(3, 3);


//Therefor we start on row 2(index 1) in the for loop
        for (int i = 1; i < row; ++i) {
            gravityVecs(i, 0) = std::stod(data[i][9]);
            gravityVecs(i, 1) = std::stod(data[i][10]);
            gravityVecs(i, 2) = std::stod(data[i][11]);

            forceVecs(i, 0) = std::stod(data[i][0]);
            forceVecs(i, 1) = std::stod(data[i][1]);
            forceVecs(i, 2) = std::stod(data[i][2]);

            AxyzStacked(0) += std::stod(data[i][6]);
            AxyzStacked(1) += std::stod(data[i][7]);
            AxyzStacked(2) += std::stod(data[i][8]);

            measurementsOfForce(0) += std::stod(data[i][0]);
            measurementsOfForce(1) += std::stod(data[i][1]);
            measurementsOfForce(2) += std::stod(data[i][2]);
            //These are for torque

            stackedTorques(0) += std::stod(data[i][3]);  // Collect columns 4-6
            stackedTorques(1) += std::stod(data[i][4]);
            stackedTorques(2) += std::stod(data[i][5]);

            torqueMeasurements(i, 0) = std::stod(data[i][3]);  // Collect columns 4-6
            torqueMeasurements(i, 1) = std::stod(data[i][4]);
            torqueMeasurements(i, 2) = std::stod(data[i][5]);

            gravityMatrix(i, 0) = std::stod(data[i][9]);
            gravityMatrix(i, 1) = std::stod(data[i][10]);
            gravityMatrix(i, 2) = std::stod(data[i][11]);
        }

        double massEstimate = estimationOfMass(gravityVecs, forceVecs);


        //This is now the correct IMU BIAS
        std::cout << "The mass estimate is: " << massEstimate << std::endl;
        auto imubias = calculateIMUBias(AxyzStacked, row-1);
        std::cout << "The imu bias is:" << imubias.transpose() << std::endl;



        //This gives the correct output:
        Eigen::VectorXd forceBias = calculateForceBias(measurementsOfForce, row-1);
        std::cout << "The force bias is: " << forceBias.transpose() << std::endl; //}



        //std::cout << "stacked torques:" << stackedTorques << std::endl;
        //std::cout << "stacked gravMatrices:" << stackedGravMatrices << std::endl;


        //auto massCenter = massCenterEstimation(stackedGravMatrices, stackedTorques, massEstimate);
        auto massCenter = estimateMassCenter(gravityMatrix, torqueMeasurements, massEstimate, row);

        std::cout << "The calculated mass center is: " << massCenter.transpose() << ", with the mass estimate of :" << massEstimate
                  << "." << std::endl;

        Eigen::Vector3d solutionMassCenter;
        massCenter << 0.00027917, 5.44378 * pow(10, -5), 0.43896;


        //THIS IS CORRECT
        Eigen::Vector3d torqueBias = calculateTorqueBias(torqueMeasurements, 0, 8);  // For tx
        std::cout << "The calculated  torque bias is: " << torqueBias.transpose() << std::endl;

        estimation::kalman_filter kf(massEstimate, massCenter, forceBias, imubias);

        for (int i = 0; i < row; ++i) {
            //må kankjse oppdatere rotasjonsmatrise
//            kf.set_rotation_matrix()
            kf.predict();
            Eigen::Vector3d imuAcceleration(std::stod(data[i][6]), std::stod(data[i][7]), std::stod(data[i][8]));
            Eigen::Vector3d ftsForce(std::stod(data[i][0]), std::stod(data[i][1]), std::stod(data[i][2]));
            Eigen::Vector3d ftsTorque(std::stod(data[i][3]), std::stod(data[i][4]), std::stod(data[i][5]));

            kf.update(imuAcceleration, ftsForce, ftsTorque);

            Eigen::VectorXd contact_wrench = kf.get_contact_wrench();

            std::cout << "Estimated contact wrench: " << contact_wrench << std::endl;
        }

        return 0;
    }