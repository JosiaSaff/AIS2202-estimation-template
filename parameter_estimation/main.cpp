#include <rapidcsv.h>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <sstream>
#include <filesystem>
#include <math.h>
#include <vector>

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


Eigen::VectorXd massCenterEstimation(Eigen::MatrixXd &gravityVecg, Eigen::VectorXd &tourqeVec, double mStar){
    return ((gravityVecg.transpose() * gravityVecg).inverse() * gravityVecg.transpose() * tourqeVec)/mStar;
}


Eigen::VectorXd estimateForceBias(const Eigen::VectorXd &measurements, int rows, int columns) {
    Eigen::VectorXd forceBias = Eigen::VectorXd::Zero(columns);

    for (int i = 0; i < columns; i++) {
        forceBias(i) = measurements(i) / rows;
    }
    return forceBias;
}

//Eigen::VectorXd findTorqueBias(Eigen::VectorXd torqueVector, Eigen::VectorXd forceVector, Eigen::VectorXd armVector)


Eigen::Vector3d calculateTorqueBias(const Eigen::MatrixXd &torqueMeasurements, int startIndex, int endIndex) {
    Eigen::Vector3d torqueBias;
    int count = endIndex - startIndex + 1;

    // Calculate the mean of the range for each torque axis (tx, ty, tz)
    torqueBias(0) = torqueMeasurements.col(0).segment(startIndex, count).mean();
    torqueBias(1) = torqueMeasurements.col(1).segment(startIndex, count).mean();
    torqueBias(2) = torqueMeasurements.col(2).segment(startIndex, count).mean();

    return torqueBias;
}

    Eigen::Vector3d calculateForceBias(const Eigen::MatrixXd &forceMeasurements, int rows){
        Eigen::Vector3d forceBias;
        forceBias(0) = forceMeasurements.col(0).mean();
        forceBias(1) = forceMeasurements.col(1).mean();
        forceBias(2) = forceMeasurements.col(2).mean();
        return forceBias;
    }


//There are 24 measurements, in a total of 25 lines in the calibration_fts dataset


    Eigen::Vector3d calculateIMUBias(const Eigen::MatrixXd &imuMeasurements, int row) {
        //Eigen::VectorXd imuBias = Eigen::VectorXd::Zero(3);
        Eigen::VectorXd imuBias(3);
        //imuBias(2) = imuMeasurements.col(2).mean();
        for(int i = 1; i < row+1; i++){
            imuBias(0) += imuMeasurements(i, 0);
            imuBias(1) += imuMeasurements(i, 1);
            imuBias(2) += imuMeasurements(i, 2);
            std::cout << "lolololololololololol for row  "  << i <<  "   :  " << imuMeasurements(i, 0) << "   " << imuMeasurements(i, 1) << "   " << imuMeasurements(i, 2) << std::endl;
        }
        for(int i = 0; i < 3; i++)
        imuBias(i) =  imuBias(i)/static_cast<float>(row);

        return imuBias;
    }


    const int MAX_ROWS = 100;
    const int MAX_COLS = 100;


    const std::string filePath = "..\\datasets\\0-calibration_fts-accel.csv";

    int main() {

        if (!std::filesystem::exists(filePath)) {
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
                std::cout << data[i][j] << " ";
            }
            std::cout << std::endl;
        }

        std::cout << "Number of rows: " << row << std::endl;
        std::cout << "Number of columns per row (max): " << MAX_COLS << std::endl;


        Eigen::MatrixXd gravityVecs = Eigen::MatrixXd::Zero(row, 3);
        Eigen::MatrixXd forceVecs = Eigen::MatrixXd::Zero(row, 3);

//    std::cout << "Values i want: " << gravityVecs(0,0) << std::endl;

        std::cout << "data[1][9] has value " << data[0][1] << " and is of type " << typeid(data[1][9]).name()
                  << std::endl;

// Of cource has to skip the first row of fx,fy,fz,tx,ty,tz,ax,ay,az,gx,gy,gz,r11,r12,r13,r21,r22,r23,r31,r32,r33


        //Eigen::MatrixXd Axyz = Eigen::MatrixXd::Zero(row, 3);
        Eigen::MatrixXd Axyz(row, 3);


//Therefor we start on row 2(index 1) in the for loop
        for (int i = 1; i < row; ++i) {
            gravityVecs(i, 0) = std::stod(data[i][9]);
            gravityVecs(i, 1) = std::stod(data[i][10]);
            gravityVecs(i, 2) = std::stod(data[i][11]);

            forceVecs(i, 0) = std::stod(data[i][0]);
            forceVecs(i, 1) = std::stod(data[i][1]);
            forceVecs(i, 2) = std::stod(data[i][2]);

            Axyz(i, 0) = std::stod(data[i][6]);
            Axyz(i, 1) = std::stod(data[i][7]);
            Axyz(i, 2) = std::stod(data[i][8]);

        }

        double massEstimate = estimationOfMass(gravityVecs, forceVecs);

        std::cout << "The mass estimate is: " << massEstimate << std::endl;

        auto imubias = calculateIMUBias(Axyz, row-1);

        std::cout << "The imu bias is:" << imubias << std::endl;

        //Type problem
        Eigen::VectorXd measurementsOfForce(3);  // Resize vector to have 'row' elements
        for (int i = 2; i < row; ++i) {
            measurementsOfForce(0) += std::stod(data[i][0]);
            measurementsOfForce(1) += std::stod(data[i][1]);
            measurementsOfForce(2) += std::stod(data[i][2]);
            //These are for torque
            std::cout << i << std::endl;
        }

        std::cout << "measurements of force matrix is: " << measurementsOfForce << std::endl;

        //std::cout << "" << std::endl;
        std::cout << "" << std::endl;

        //this does not work for some reason{
        Eigen::VectorXd forceBias = calculateForceBias(measurementsOfForce);

        //This is correct
        std::cout << "The force bias is: " << forceBias << std::endl; //}






        Eigen::MatrixXd torqueMeasurements = Eigen::MatrixXd::Zero(row, 3);
        Eigen::VectorXd stackedTorques = Eigen::VectorXd::Zero(3);
        Eigen::MatrixXd stackedGravMatrices = Eigen::MatrixXd::Zero(3, 3);

        //  Eigen:  : M  a    t r i x X  d gravityVec(row, 3);
        //The matrix is not initialized at 0 for some reason

        for (int i = 0; i < 3; ++i) {
            stackedTorques(i) = 0.0;
            for (int j = 0; j < 3; ++j) {
                stackedGravMatrices(i, j) = 0.0;

            }
        }
        std::cout << "stacked graxMatrices:" << stackedGravMatrices << std::endl;


        for (int i = 1; i < row; ++i) {
            stackedTorques(0) += std::stod(data[i][3]);  // Collect columns 4-6
            stackedTorques(1) += std::stod(data[i][4]);
            stackedTorques(2) += std::stod(data[i][5]);

            torqueMeasurements(i, 0) = std::stod(data[i][3]);  // Collect columns 4-6
            torqueMeasurements(i, 1) = std::stod(data[i][4]);
            torqueMeasurements(i, 2) = std::stod(data[i][5]);


            stackedGravMatrices(1, 0) += std::stod(data[i][11]);
            stackedGravMatrices(0, 2) += std::stod(data[i][10]);
            stackedGravMatrices(2, 1) += std::stod(data[i][9]);
        }

        //I have to make sure the indexes are mirrored

        stackedGravMatrices(0, 1) = -stackedGravMatrices(1, 0); //this is for the secoudnd
        stackedGravMatrices(2, 0) = -stackedGravMatrices(0, 2); // this is for the first
        stackedGravMatrices(1, 2) = -stackedGravMatrices(2, 1); //This is cenrtantly for the third


        std::cout << "stacked torques:" << stackedTorques << std::endl;
        std::cout << "stacked gravMatrices:" << stackedGravMatrices << std::endl;


        auto massCenter = massCenterEstimation(stackedGravMatrices, stackedTorques, massEstimate);

        std::cout << "" << std::endl;
        std::cout << "The calculated mass center is: " << massCenter << ", with the mass estimate of :" << massEstimate
                  << "." << std::endl;
        std::cout << " ------------------------------------------------------------------------- " << std::endl;
        std::cout << " ------------------------------------------------------------------------- " << std::endl;

        // I can't reach the documentation WHAT!!??!?!?!?

        Eigen::Vector3d solutionMassCenter;
        massCenter << 0.00027917, 5.44378 * pow(10, -5), 0.43896;


        //THIS IS CORREC
        //
        Eigen::Vector3d torqueBias = calculateTorqueBias(torqueMeasurements, 0, 7);  // For tx
        Eigen::Vector3d tyBias = calculateTorqueBias(torqueMeasurements, 8, 15);  // For ty
        Eigen::Vector3d tzBias = calculateTorqueBias(torqueMeasurements, 16, 23);  // For tz


        std::cout << "" << std::endl;
        std::cout << "The calculated  torque bias is: " << torqueBias << std::endl;
        std::cout << " ------------------------------------------------------------------------- " << std::endl;
        std::cout << " ------------------------------------------------------------------------- " << std::endl;


        return 0;
    }

//rotasjonsmatrise for tilstandsestimering på nettsida

//Les første del i paper 2 til onsdag

// Ai = [0 igz igy][giz 0 giz][igy -igx 0]