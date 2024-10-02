#include <rapidcsv.h>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <sstream>
#include <filesystem>
#include <vector>

// https://www.ibm.com/docs/en/zos/2.4.0?topic=only-explicit-initialization-constructors-c
class parameterEstimator {
    double re, im;
public:

    // default constructor
    parameterEstimator() : re(0), im(0) { }

    // copy constructor
    parameterEstimator(const parameterEstimator& c) { re = c.re; im = c.im; }

    // constructor with default trailing argument
    parameterEstimator( double r, double i = 0.0) { re = r; im = i; }

    void display() {
        std::cout << "re = "<< re << " im = " << im << std::endl;
    }
};



//Make sure parameter_estimation folder is used


Eigen::MatrixXd pseudo_inverse(const Eigen::MatrixXd &G, const Eigen::MatrixXd &F)
{
    auto pseudoInv = (G.transpose() * G).completeOrthogonalDecomposition().pseudoInverse() * (G.transpose() * F);
    return pseudoInv;
}

double estimationOfMass(Eigen::MatrixXf &gravityVecs, Eigen::MatrixXf &forceVecs){
    return ((gravityVecs.transpose() * gravityVecs).completeOrthogonalDecomposition().pseudoInverse() * (gravityVecs.transpose() * forceVecs)).value();
}


Eigen::VectorXd massCenterEstimation(const Eigen::MatrixXd &gravityVecg, const Eigen::MatrixXd &tourqeVec, double mStar){
    return (1/mStar)*(gravityVecg.transpose() * gravityVecg).inverse() * gravityVecg.transpose() * tourqeVec;
}


/*
 * Eigen::VectorXd estimateForceBias(Eigen::MatrixXf measurements) {
    Eigen::VectorXd forceBias = Eigen::VectorXd::Zero(6);
    for (const auto& measurement : measurements) {
        forceBias += measurement;
    }
    forceBias = forceBias / measurements.size();
    return forceBias;
}*/


//There are 24 measurements, in a total of 25 lines in the calibration_fts dataset



Eigen::MatrixXd gravityCompensation(const Eigen::VectorXd& gravityVec, const Eigen::MatrixXd& rotMatrix, double massEstimated, const Eigen::VectorXd &torqueVec) {
    Eigen::Vector3d gravityForce = massEstimated * (rotMatrix.transpose() * gravityVec);
    Eigen::Vector3d centerOfMass = massCenterEstimation(gravityVec, torqueVec, massEstimated);

    Eigen::Vector3d gravityTorque = centerOfMass.cross(gravityForce);
    Eigen::MatrixXd compensation(6, 1);
    compensation.block<3,1>(0,0) = gravityForce;
    compensation.block<3,1>(3,0) = gravityTorque;
    return compensation;
}


const int MAX_ROWS = 100;
// Maximum number of columns for the 2D array
const int MAX_COLS = 100;


float fxTot = 0.0;
float fyTot = 0.0;
float fzTot = 0.0;

float txTot = 0.0;
float tyTot = 0.0;
float tzTot = 0.0;

float axTot = 0.0;
float ayTot = 0.0;
float azTot = 0.0;

float gxTot = 0.0;
float gyTot = 0.0;
float gzTot = 0.0;

float r11Tot = 0.0;
float r12Tot = 0.0;
float r13Tot = 0.0;

float r21Tot = 0.0;
float r22Tot = 0.0;
float r23Tot = 0.0;

float r31Tot = 0.0;
float r32Tot = 0.0;
float r33Tot = 0.0;



const std::string filePath = "C:\\Users\\hanur\\CLionProjects\\Cybernetics\\AIS2202-estimation-template\\datasets\\0-calibration_fts-accel.csv";

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


    Eigen::MatrixXf gravityVecs(row, 3);
    Eigen::MatrixXf forceVecs(row, 3);

//    std::cout << "Values i want: " << gravityVecs(0,0) << std::endl;

    std::cout << "data[1][9] has value " << data[0][1] << " and is of type " << typeid(data[1][9]).name() << std::endl;

// Of cource has to skip the first row of fx,fy,fz,tx,ty,tz,ax,ay,az,gx,gy,gz,r11,r12,r13,r21,r22,r23,r31,r32,r33



//Therefor we start on row 2(index 1) in the for loop
    for (int i = 1; i < row; ++i) {
        gravityVecs(i,0) = std::stof(data[i][9]);
        gravityVecs(i,1) = std::stof(data[i][10]);
        gravityVecs(i,2) = std::stof(data[i][11]);

        forceVecs(i,0) = std::stof(data[i][0]);
        forceVecs(i,1) = std::stof(data[i][1]);
        forceVecs(i,2) = std::stof(data[i][2]);
    }

    double massEstimate = estimationOfMass(gravityVecs, forceVecs);

    std::cout << "The mass estimate is: " << massEstimate << std::endl;



/*

    Eigen::MatrixXf measurementsOfForce(row, 3);  // Resize vector to have 'row' elements
    for (int i = 0; i < row; ++i) {
        //Eigen::VectorXd measurement(6);
        measurementsOfForce(i,0) = std::stof(data[i][0]);
        measurementsOfForce(i,1) = std::stof(data[i][1]);
        measurementsOfForce(i,2) = std::stof(data[i][2]);

        measurementsOfForce(i,3) = std::stof(data[i][3]);
        measurementsOfForce(i,4) = std::stof(data[i][4]);
        measurementsOfForce(i,5) = std::stof(data[i][5]);
    }
    Eigen::VectorXd forceBias = estimateForceBias(measurementsOfForce);

    std::cout << "The force bias is: " << forceBias << std::endl;






    Eigen::MatrixXd torqueVec(row, 3);
    //  Eigen:  : M  a    t r i x X  d gravityVec(row, 3);

    for (int i = 0; i < row; ++i) {
        torqueVec.row(i) << std::stod(data[i][3]), std::stod(data[i][4]), std::stod(data[i][5]);   // Collect columns 4-6
        //gravityVec.row(i) << std::stod(data[i][9]), std::stod(data[i][10]), std::stod(data[i][11]);  // Collect columns 10-12
    }

    //auto massCenter = massCenterEstimation(gravityVec, torqueVec, massEstimate);


    //std::cout << "The mass center is: " << massCenter << std::endl;



*/
    return 0;
}

//rotasjonsmatrise for tilstandsestimering på nettsida

//Les første del i paper 2 til onsdag

// Ai = [0 igz igy][giz 0 giz][igy -igx 0]
