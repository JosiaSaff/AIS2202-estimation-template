clc, close all, clear all;
% For the last task.

% Load the FTS and IMU data
FTS_data = readtable('../datasets/0-calibration_fts-accel.csv');
steady_state_accel = readtable('..\datasets\0-steady-state_accel.csv');
steady_state_wrench = readtable('..\datasets\0-steady-state_wrench.csv');

baseline_accel = readtable('..\datasets\2-vibrations_accel.csv');
baseline_orientations = readtable('..\datasets\2-vibrations_orientations.csv');
baseline_wrench = readtable('..\datasets\1-baseline_wrench.csv');

vibrations_accel = readtable('datasets\2-vibrations_accel.csv');
vibrations_wrench = readtable('datasets\2-vibrations_wrench.csv');
vibrations_orientations = readtable('datasets\2-vibrations_orientations.csv');

vib_contact_accel = readtable('datasets\3-vibrations-contact_accel.csv');
vib_contact_wrench = readtable('datasets\3-vibrations-contact_wrench.csv');
vib_contact_orientations = readtable('datasets\3-vibrations-contact_orientations.csv');

% Align datasets by their timestamps (assuming they all have a "t" column for time)
% Find the earliest timestamp and synchronize each dataset accordingly
baseline_time = baseline_accel.t - min(baseline_accel.t);
vibration_time = vibrations_accel.t - min(vibrations_accel.t);
vib_contact_time = vib_contact_accel.t - min(vib_contact_accel.t);

% Extract force and torque from FTS_data;
fx = FTS_data.fx;
fy = FTS_data.fy;
fz = FTS_data.fz;
tx = FTS_data.tx;
ty = FTS_data.ty;
tz = FTS_data.tz;

% Extract accelerometer data
ax = FTS_data.ax;
ay = FTS_data.ay;
az = FTS_data.az;

% Extract gravity direction (gx, gy, gz) - used later for compensation
gx = FTS_data.gx;
gy = FTS_data.gy;
gz = FTS_data.gz;

% Estimate mass and mass center (from the paper, eq. (23))
estimated_mass = 0.932308;

%mass center shown in paper
mass_center = [0.0, 0, 0.0439107]; 

% Compute gravity compensation vector (from eq. (12) in the paper)
g_w = [0; 0; -9.81];  % Gravity vector in world frame


R_fs = [0, 0, -1; -1, 0, 0; 0, 1, 0];  

g_s = R_fs * g_w;  % Gravity in sensor frame

%Equation (12)
Vg = [estimated_mass * g_s; cross(mass_center', estimated_mass * g_s')'];

variance_f_torque = [0.3090 0.1110 1.4084]
variance_f_force = [0.0068 0.0175 0.0003]
variance_f_acceleration = [0.4193 0.1387 0.9815]

% Gravity vector Vg
% Subtract gravity from the wrench measurements
compensated_fx = fx - Vg(1);
compensated_fy = fy - Vg(2);
compensated_fz = fz - Vg(3);
compensated_tx = tx - Vg(4);
compensated_ty = ty - Vg(5);
compensated_tz = tz - Vg(6);

% Compute the variances for force and acceleration (steady state)
%force_variances = var([steady_state_wrench.fx, steady_state_wrench.fy, steady_state_wrench.fz]);
%accel_variances = var([steady_state_accel.ax, steady_state_accel.ay, steady_state_accel.az]);

disp('Force Variances:');
disp(force_variances);

disp('Accelerometer Variances:');
disp(accel_variances);

% Kalman Filter Initialization
dt = 0.001;  % Time step, assuming 1 kHz sampling rate
n = length(baseline_accel.ax);  % Assume same length for accel and wrench data

% Initial state for forces and torques
X = zeros(6, n);  % State vector [fx; fy; fz; tx; ty; tz]
P = eye(6);       % Covariance matrix
Q = blkdiag(eye(3), eye(3));  % Process noise covariance (assuming identity for simplicity)

% Assume R (measurement noise covariance) calculated from variance of steady-state data
R_f = [diag(variance_f_force), zeros(3,3); zeros(3,3) diag(variance_f_torque)];

R_a = diag(variance_f_acceleration)
% Kalman filter loop for
for i = 2:n
    % Prediction step
    X(:, i) = X(:, i-1); 
    P = P + Q;            % Update covariance matrix

    % Measurement vector
    Z = [baseline_wrench.fx(i); baseline_wrench.fy(i); baseline_wrench.fz(i); ...
         baseline_wrench.tx(i); baseline_wrench.ty(i); baseline_wrench.tz(i)];

    % Kalman gain
    K = P / (P + R_f);  % Using force/torque measurement noise

    % Update step
    X(:, i) = X(:, i) + K * (Z - X(:, i));
    P = (eye(6) - K) * P;
end



% Apply the same process for the vibrations and vibrations-contact cases
% Replace baseline data with the respective datasets
% e.g. vibrations_accel, vibrations_wrench, and vibrations_orientations

% Implement the same Kalman filter process for vibrations and contact conditions
% Similar loops for 'vibrations_*' and 'vib_contact_*' datasets


% Extract the estimated forces and torques
estimated_fx = X(1, :);
estimated_fy = X(2, :);
estimated_fz = X(3, :);
estimated_tx = X(4, :);
estimated_ty = X(5, :);
estimated_tz = X(6, :);

% Plot the estimated forces for baseline case
figure;
subplot(3,1,1);
plot(X(1, :)); title('Estimated Force in X (Baseline)');
subplot(3,1,2);
plot(X(2, :)); title('Estimated Force in Y (Baseline)');
subplot(3,1,3);
plot(X(3, :)); title('Estimated Force in Z (Baseline)');

% Plot for the vibrations-contact case (you can repeat for the others)
figure;
subplot(3,1,1);
plot(X(1, :)); title('Estimated Force in X (Vibrations + Contact)');
subplot(3,1,2);
plot(X(2, :)); title('Estimated Force in Y (Vibrations + Contact)');
subplot(3,1,3);
plot(X(3, :)); title('Estimated Force in Z (Vibrations + Contact)');
