clc, close all, clear all;
FTS_data = readtable('datasets/0-calibration_fts-accel.csv');
SSaccel = readtable('datasets/0-steady-state_accel.csv');
SSwrench = readtable('datasets/0-steady-state_wrench.csv');

BLAccel = readtable('datasets/1-baseline_accel.csv');
BLOrients = readtable('datasets/1-baseline_orientations.csv');
BLWrench = readtable('datasets/1-baseline_wrench.csv');

VibsAccel = readtable('datasets/2-vibrations_accel.csv');
VibsWrench = readtable('datasets/2-vibrations_wrench.csv');
VibsOrientations = readtable('datasets/2-vibrations_orientations.csv');
%Vibration contact
vibContAccel = readtable('datasets/3-vibrations-contact_accel.csv');
vibContWrench = readtable('datasets/3-vibrations-contact_wrench.csv');
vibContOrients = readtable('datasets/3-vibrations-contact_orientations.csv');



%This is for objective 1:
%Find variances

SSaccel.ax = SSaccel.ax*9.81;
SSaccel.ay = SSaccel.ay*9.81;
SSaccel.az = SSaccel.az*9.81;




varAccelAx = var(SSaccel.ax);
varAccelAy = var(SSaccel.ay);
varAccelAz = var(SSaccel.az);


varWrenchFx = var(SSwrench.fx);
varWrenchFy = var(SSwrench.fy);
varWrenchFz = var(SSwrench.fz);

varWrenchTx = var(SSwrench.tx);
varWrenchTy = var(SSwrench.ty);
varWrenchTz = var(SSwrench.tz);
%checked that i get the right answer for both force and torque.

varianceVec = [([varAccelAx, varAccelAy, varAccelAz]*100),...
    ([varWrenchFx, varWrenchFy, varWrenchFz]*250),...
    ([varWrenchTx, varWrenchTy, varWrenchTz]*5000)];

disp('variance vector:')
disp(varianceVec)


n = length(BLWrench.fx); 

%mass, mass center, biases and such
mass = 0.932; %kg
massCenter = [0, 0, 0.044];

massCenterScrewsym = [0 -massCenter(3) massCenter(2);
    massCenter(3) 0 -massCenter(1);
    -massCenter(2) massCenter(1) 0 ];

Rfs = [0, -1, 0; 0, 0, 1; -1, 0, 0];
gravityVec = [0; 0; -9.81];

gs = Rfs * gravityVec; 

imuBias = [-0.00366194  0.00884945   0.0771078]
forceBias =  [9.07633 -1.01814  9.98482]
torqueBias =  [0.432449, -0.692162, -0.156746]




nStates = 9;
measurement_dim = 9;

Vg = [mass * gs; mass*cross(massCenter', gs')']



%gaussianNoiceVector
gaussianNoiceVec = [sqrt(varAccelAx), sqrt(varAccelAy), sqrt(varAccelAz),...
    sqrt(varWrenchFx), sqrt(varWrenchFy), sqrt(varWrenchFz),...
    sqrt(varWrenchTx), sqrt(varWrenchTy), sqrt(varWrenchTz)];

% Bias handeling for all datasets


%This is for the BaseLine dataset
BLAccel.ax = BLAccel.ax - imuBias(1);
BLAccel.ay = BLAccel.ay - imuBias(2);
BLAccel.az = BLAccel.az - imuBias(3);
VibsAccel.ax = VibsAccel.ax - imuBias(1);
VibsAccel.ay = VibsAccel.ay - imuBias(2);
VibsAccel.az = VibsAccel.az - imuBias(3);
vibContAccel.ax = vibContAccel.ax - imuBias(1);
vibContAccel.ay = vibContAccel.ay - imuBias(2);
vibContAccel.az = vibContAccel.az - imuBias(3);


BLWrench.fx = BLWrench.fx - forceBias(1) - Vg(1);
BLWrench.fy = BLWrench.fy - forceBias(2) - Vg(2);
BLWrench.fz = BLWrench.fz - forceBias(3) - Vg(3);
BLWrench.tx = BLWrench.tx - torqueBias(1) - Vg(4);
BLWrench.ty = BLWrench.ty - torqueBias(2) - Vg(5);
BLWrench.tz = BLWrench.tz - torqueBias(3) - Vg(6);

VibsWrench.fx = VibsWrench.fx - forceBias(1) - Vg(1);
VibsWrench.fy = VibsWrench.fy - forceBias(2) - Vg(2);
VibsWrench.fz = VibsWrench.fz - forceBias(3) - Vg(3);
VibsWrench.tx = VibsWrench.tx - torqueBias(1) - Vg(4);
VibsWrench.ty = VibsWrench.ty - torqueBias(2) - Vg(5);
VibsWrench.tz = VibsWrench.tz - torqueBias(3) - Vg(6);

vibContWrench.fx = vibContWrench.fx - forceBias(1) - Vg(1);
vibContWrench.fy = vibContWrench.fy - forceBias(2) - Vg(2);
vibContWrench.fz = vibContWrench.fz - forceBias(3) - Vg(3);
vibContWrench.tx = vibContWrench.tx - torqueBias(1) - Vg(4);
vibContWrench.ty = vibContWrench.ty - torqueBias(2) - Vg(5);
vibContWrench.tz = vibContWrench.tz - torqueBias(3) - Vg(6);


BLAccel.ax = BLAccel.ax * -9.81; 
BLAccel.ay = BLAccel.ay * -9.81;
BLAccel.az = BLAccel.az * -9.81;

VibsAccel.ax = VibsAccel.ax * -9.81;
VibsAccel.ay = VibsAccel.ay * -9.81;
VibsAccel.az = VibsAccel.az * -9.81;

vibContAccel.ax = vibContAccel.ax * -9.81;
vibContAccel.ay = vibContAccel.ay * -9.81;
vibContAccel.az = vibContAccel.az * -9.81;

% KALMAN FILTER FOR BASELINE DATA
%Run the first cell right before you run this one 
%RFS is right for baseline data
Rfs = [0, -1, 0; 0, 0, 1; -1, 0, 0];
gs = Rfs * gravityVec; 

%Removing biases befor processing data through the Kalman Filter:


%Iniatilization of Kalman
blKF = KalmanFilter(measurement_dim, mass, massCenterScrewsym, [forceBias, torqueBias], ...
    massCenter, BLAccel, BLWrench ,gaussianNoiceVec, varianceVec);

% Initialize Fusion object
blFusion = Fusion(blKF);

% Load data


blFusion = blFusion.setupStateSpace(mass, massCenter, Rfs, gravityVec, n, forceBias, torqueBias, Vg);
%run(obj, wrenchDataset, accelDataset, orientationsDataset, lastTrajectory,wrench, orien)
blFusion.run(BLWrench, BLAccel, BLOrients, Rfs);
%fusion.plot();


%% KALMAN FILTER FOR VIBRATIONS
%Iniatilization of Kalman

%Check if this is correct for all
Rfs = [0, -1, 0; 0, 0, 1; -1, 0, 0];
gs = Rfs * gravityVec; 



vibsKF = KalmanFilter(measurement_dim, mass, massCenterScrewsym, [forceBias, torqueBias], ...
    massCenter, VibsAccel, VibsWrench ,gaussianNoiceVec, varianceVec);

% Initialize Fusion object
vibsFusion = Fusion(vibsKF);

% Load data

% Set up state-space model

            %setupStateSpace(obj, mass, mass_center, Rfs, gravityVec)
vibsFusion = vibsFusion.setupStateSpace(mass, massCenter, Rfs, gravityVec, n, forceBias, torqueBias, Vg);
%fusion.% Run Fusion
%run(obj, wrenchDataset, accelDataset, orientationsDataset, lastTrajectory,wrench, orien)
vibsFusion.run(VibsWrench, VibsAccel, VibsOrientations, Rfs);


%% KALMAN FILTER FOR VIBRATIONS CONTACT WRENCH
%Run the first script
%Iniatilization of Kalman

%Check if this is correct for all
Rfs = [0, -1, 0; 0, 0, 1; -1, 0, 0];
gs = Rfs * gravityVec; 


vibContantKF = KalmanFilter(measurement_dim, mass, massCenterScrewsym, [forceBias, torqueBias], ...
    massCenter, vibContAccel, vibContWrench ,gaussianNoiceVec, varianceVec);

% Initialize Fusion object
vibsContactFusion = Fusion(vibContantKF);



vibsContactFusion = vibsContactFusion.setupStateSpace(mass, massCenter, Rfs, gravityVec, n, forceBias, torqueBias, Vg);



%Running
vibsContactFusion.run(vibContWrench, vibContAccel, vibContOrients, Rfs);






