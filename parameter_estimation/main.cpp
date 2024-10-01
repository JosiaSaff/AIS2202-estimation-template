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





    std::vector<Eigen::VectorXd> measurementsOfForce;
    for (int i = 0; i < row; ++i) {
        Eigen::VectorXd measurement(6);
        measurement << std::stod(data[i][0]), std::stod(data[i][1]), std::stod(data[i][2]),  // Fx, Fy, Fz
                std::stod(data[i][3]), std::stod(data[i][4]), std::stod(data[i][5]);  // Tx, Ty, Tz
        measurementsOfForce.push_back(measurement);
    }
    Eigen::VectorXd forceBias = estimateForceBias(measurementsOfForce);

    std::cout << "The force bias is: " << forceBias << std::endl;


    Eigen::MatrixXd gravityVec(row, 3);
    Eigen::MatrixXd forceVec(row, 3);

    for (int i = 0; i < row; ++i) {
        gravityVec.row(i) << std::stod(data[i][9]), std::stod(data[i][10]), std::stod(data[i][11]);
        forceVec.row(i) << std::stod(data[i][0]), std::stod(data[i][1]), std::stod(data[i][2]);
    }

    double massEstimate = estimationOfMass(gravityVec, forceVec);

    std::cout << "The mass estimate is: " << massEstimate << std::endl;



    Eigen::MatrixXd torqueVec(row, 3);
    // /* Eigen:  : M  a    t r i x X  d gravityVec(row, 3); */

    for (int i = 0; i < row; ++i) {
        torqueVec.row(i) << std::stod(data[i][3]), std::stod(data[i][4]), std::stod(data[i][5]);   // Collect columns 4-6
        //gravityVec.row(i) << std::stod(data[i][9]), std::stod(data[i][10]), std::stod(data[i][11]);  // Collect columns 10-12
    }

    Eigen::VectorXd massCenter = massCenterEstimation(gravityVec, torqueVec, massEstimate);


    std::cout << "The mass center is: " << massCenter << std::endl;





    return 0;
}
