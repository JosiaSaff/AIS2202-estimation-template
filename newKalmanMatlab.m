clc, close all, clear all;
% For the last task.

% Load the FTS and IMU data
FTS_data = readtable('datasets/0-calibration_fts-accel.csv');
steady_state_accel = readtable('datasets/0-steady-state_accel.csv');
steady_state_wrench = readtable('datasets/0-steady-state_wrench.csv');

baseline_accel = readtable('datasets/1-baseline_accel.csv');
baseline_orientations = readtable('datasets/1-baseline_orientations.csv');
baseline_wrench = readtable('datasets/1-baseline_wrench.csv');

vibrations_accel = readtable('datasets/2-vibrations_accel.csv');
vibrations_wrench = readtable('datasets/2-vibrations_wrench.csv');
vibrations_orientations = readtable('datasets/2-vibrations_orientations.csv');

vib_contact_accel = readtable('datasets/3-vibrations-contact_accel.csv');
vib_contact_wrench = readtable('datasets/3-vibrations-contact_wrench.csv');
vib_contact_orientations = readtable('datasets/3-vibrations-contact_orientations.csv');

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
ax = -0.0175781;
ay = -0.993652;
az = 0.0722656;

% Extract gravity direction (gx, gy, gz) - used later for compensation
gx = FTS_data.gx;
gy = FTS_data.gy;
gz = FTS_data.gz;



fx_baseline = baseline_wrench.fx;
fy_baseline = baseline_wrench.fy;
fz_baseline = baseline_wrench.fz;
tx_baseline = baseline_wrench.tx;
ty_baseline = baseline_wrench.ty;
tz_baseline = baseline_wrench.tz;
wrench_ss_time = baseline_wrench.t;
orientations_ss_time = baseline_orientations.t
% Estimate mass and mass center (from the paper, eq. (23))

%Finding standard deviations
std_ax = std(baseline_accel.ax);
std_ay = std(baseline_accel.ay);
std_az = std(baseline_accel.az);
std_fx = std(baseline_wrench.fx);
std_fy = std(baseline_wrench.fy);
std_fz = std(baseline_wrench.tx);
std_tx = std(baseline_wrench.tx);
std_ty = std(baseline_wrench.ty);
std_tz = std(baseline_wrench.tz);


orient_r11 = baseline_orientations.r11;
orient_r12 = baseline_orientations.r12;
orient_r13 = baseline_orientations.r13;
orient_r21 = baseline_orientations.r21;
orient_r22 = baseline_orientations.r22;
orient_r23 = baseline_orientations.r23;
orient_r31 = baseline_orientations.r31;
orient_r32 = baseline_orientations.r32;
orient_r33 = baseline_orientations.r33;


estimated_mass = 0.932308;

%mass center shown in paper
mass_center = [0.0, 0, 0.0439107]; 

mass_center_screwsym = [0 -mass_center(3) mass_center(2);
    mass_center(3) 0 -mass_center(1);
    -mass_center(2) mass_center(1) 0 ];

%THis represents the cange in the gravitational acceleration vector 
%In th sensor frame s between two consecutive time steps k and k-1




% Compute gravity compensation vector (from eq. (12) in the paper)
g_w = [0; 0; -9.81];  % Gravity vector in world frame

%the following rotation matrix R_fs which can be used to transform the acceleration measurements from the IMU frame {a} to the FTS frame {f}. 
%R_fs = [0, 1, 0; 0, 0, -1; -1, 0, 0];
R_fs = [0, -1, 0; 0, 0, 1; -1, 0, 0];
% is very simular to the vector in the dataset:
% baseline orientations


%while excample 
%while n > 1
%    n = n-1;
%    f = f*n;
%end

g_s = R_fs * g_w; 
lastTrajectory = R_fs * g_w;

%Equation (12)
%The effects of gravity on the FTS measurement
Vg = [estimated_mass * g_s; estimated_mass*cross(mass_center', g_s')']

% gravitasjonsvector sett fra sensor
% legger til ny gravitasjon og trekker fra den gamle
% representasjon av gravitasjon i verden endrer seg
%*var feil skreve

%skal gange H_f med vår tilstandsvector
%skal være et stimat for

%Equation (24)
ff = 698.3 %hz
fr = 100.2 %hz
fa = 254.3

%
A = eye(9)



variance_f_torque = [0.3090 0.1110 1.4084]
variance_f_force = [0.0068 0.0175 0.0003]
variance_f_acceleration = [0.4193 0.1387 0.9815]

% Gravity vector Vg
% Subtract gravity from the wrench measurements
compensated_fx_baseline = fx_baseline - Vg(1);
compensated_fy_baseline = fy_baseline - Vg(2);
compensated_fz_baseline = fz_baseline - Vg(3);
compensated_tx_baseline = tx_baseline - Vg(4);
compensated_ty_baseline = ty_baseline - Vg(5);
compensated_tz_baseline = tz_baseline - Vg(6);

% Compute the variances for force and acceleration (steady state)
%force_variances = var([steady_state_wrench.fx, steady_state_wrench.fy, steady_state_wrench.fz]);
%accel_variances = var([steady_state_accel.ax, steady_state_accel.ay, steady_state_accel.az]);


dt_mean = mean(diff(baseline_wrench.t)) * 1e-6;
linearAccelerationVariance = 0.5; %Specified in equation 24 in paper 3

%Finding how many times the code should iterate

n = length(baseline_wrench.fx); 
H_a = [eye(3), zeros(3), zeros(3)];
H_f = [zeros(3), eye(3), zeros(3);
    zeros(3), zeros(3), eye(3)];
H_k = [H_a; H_f]

H_c = [-estimated_mass * eye(3), eye(3), zeros(3);
       -estimated_mass * mass_center_screwsym, zeros(3), eye(3)];
%H_c = H_k

%For slicing: since all the acceleration vectors don't have the same length
%as the
len_acc = length(baseline_accel.ax);
X_f_t = [compensated_fx_baseline, compensated_fy_baseline, compensated_fz_baseline, compensated_tx_baseline, compensated_ty_baseline, compensated_tz_baseline];
zHat_cont = zeros(n, 6);
X_hat_est = zeros(n, 6);
X = zeros(n, 9);
z_k = zeros(n, 9)
% Initial state for forces and torques
%Need to transpose the line for every iteration
%X = zeros(n, 9)  
P = eye(9);

%Covariance matrix in equation 17 in paper 3 


%have to apply dynamic DT later.
Q_k = [eye(3), zeros(3), zeros(3);
  zeros(3), estimated_mass*eye(3), zeros(3);
    zeros(3), zeros(3), estimated_mass*norm(mass_center)*eye(3)]*linearAccelerationVariance;  



R_f = [diag(variance_f_force), zeros(3,3); zeros(3,3) diag(variance_f_torque)];

R_a = diag(variance_f_acceleration);

B = [eye(3); estimated_mass*eye(3); estimated_mass*mass_center_screwsym];
disp('B')
disp(B)

%D_k = zeros(size(B));
D_k = 0.1 * [eye(3); zeros(3); zeros(3)];
%D_k = diag([bias_fx, bias_fy, bias_fz, bias_tx, bias_ty, bias_tz]);

% use change in the input and the oriantation matrix
% g_w is the gravitational acceleration expressed in the world frame


deltaG_sk = g_s;
   
u_k = deltaG_sk * (fr / (ff + fa));

orientations_ss_time_index = 1;
base_accel_index = 1



for i = 2:n
    %to get toqrue and force we cant divide by mass here
    %Mass and the arm gets included by the input matrix B

    %sensor fusion

    while (wrench_ss_time(i) >  orientations_ss_time(orientations_ss_time_index)) & (orientations_ss_time_index < 627)
        %makes sure to update the trajectory
        orientations_ss_time_index = orientations_ss_time_index + 1; 
        % g_s is gravitation vector in world frave 
        deltaG_sk = ([orient_r11(orientations_ss_time_index), orient_r12(orientations_ss_time_index), orient_r13(orientations_ss_time_index); orient_r21(orientations_ss_time_index), orient_r22(orientations_ss_time_index), orient_r23(orientations_ss_time_index); orient_r31(orientations_ss_time_index), orient_r32(orientations_ss_time_index), orient_r33(orientations_ss_time_index)]*g_w - lastTrajectory);
        u_k = deltaG_sk * (fr / (ff + fa));
        lastTrajectory = [orient_r11(orientations_ss_time_index), orient_r12(orientations_ss_time_index), orient_r13(orientations_ss_time_index); orient_r21(orientations_ss_time_index), orient_r22(orientations_ss_time_index), orient_r23(orientations_ss_time_index); orient_r31(orientations_ss_time_index), orient_r32(orientations_ss_time_index), orient_r33(orientations_ss_time_index)]*g_w;
    end 

    while (wrench_ss_time(i) >  baseline_accel.t(base_accel_index)) & (base_accel_index < 1593)
        
        base_accel_index = base_accel_index + 1; 
      
        ax = baseline_accel.ax(base_accel_index);
        ay = baseline_accel.ay(base_accel_index);
        az = baseline_accel.az(base_accel_index);
    end 
    
    
    u_k = deltaG_sk * (fr / (ff + fa));
    %Measurements, or observations from the process, are related to the state vector as shown in (2)
    %define gaussian noice
    %gaussian_noise = std_dev * randn(size(u_k));
    gaussian_noise = [std_ax, std_ay, std_az, std_fx, std_fy, std_fz, std_tx, std_ty, std_tz]' .* (randn(9, 1)/10);
    %disp(H_k*u_k + B_K*u_k + [std_ax, std_ay; std_az; std_fx; std_fy; std_fz; std_tx; std_ty; std_tz] * randn(size(u_k)))
    z_k(i, :) = (H_k*[ax,ay,az,X_f_t(i,:)]' + D_k*u_k + gaussian_noise)';
  %Equation (2) in paper 3
    
    
    
    %U sjak være gravitasjonsvektor i sensorramma
    %frekvens skalering
    % u skal beskrive gravitasjonen
    % 10 ganger raskere viss vi ikke skalerer den

    %Using dynamic dt
    dt = wrench_ss_time(i) - wrench_ss_time(i-1); 
    
    % Prediction 
    %X(i, :)
    %X(i, :) = X(i-1, :); %+ B *u_k'; %Equation 5 for  Previous state propagated to next time step
    %The change in three forces is multiplyed withe the B vector. What will i get??
    %%Does this actually make any sence??
    X(i, :) = (A*X(i-1, :)' + B*u_k)';
    % X(i, :) = X(i-1, :)' + B_k * u_k;
    P = A*P*A' +  dt*Q_k;  % Equation 6 for updating the process covariance matrix

    % Measurement vector: Combine wrench and acceleration data
    %disp(i)
    Z = [ax;
     ay;
     az;
     baseline_wrench.fx(i) - Vg(1);
     baseline_wrench.fy(i) - Vg(2);
     baseline_wrench.fz(i) - Vg(3);
     baseline_wrench.tx(i) - Vg(4);
     baseline_wrench.ty(i) - Vg(5);
     baseline_wrench.tz(i) - Vg(6)];
     
    % Kalman gain: Use combined measurement noise covariance
    K = P * eye(9)' / (eye(9)*P*eye(9)' + blkdiag(R_f, R_a));  % 9x9 %Equation 7


    % Correcting the predicted state with new measurements
    X(i, :) = X(i, :)' + K * (z_k(i, :)' - X(i, :)'); %Equation 8
    
    % Update covariance matrix
    P = (eye(9) - K) * P; %Equation 9
    
    %This is the estimation model:
    
    %h_c should be correct???
    %estimation model in equation (22)
    zHat_cont(i, :) = (H_c * X(i, :)')';


end



% Apply the same process for the vibrations and vibrations-contact cases
% Replace baseline data with the respective datasets
% e.g. vibrations_accel, vibrations_wrench, and vibrations_orientations

% Implement the same Kalman filter process for vibrations and contact conditions
% Similar loops for 'vibrations_*' and 'vib_contact_*' datasets


% Extract the estimated forces and torques
estimated_ax = X(:, 1);
estimated_ay = X(:, 2);
estimated_az = X(:, 3);
estimated_fx = X(:, 4);
estimated_fy = X(:, 5);
estimated_fz = X(:, 6);
estimated_tx = X(:, 7);
estimated_ty = X(:, 8);
estimated_tz = X(:, 9);



% %Plotting
%Estimated Contact Force

figure;
subplot(3,1,1);
plot(zHat_cont(:,1), 'r');
title('Estimated Contact Force X');
xlabel('Time Step');
ylabel('Force (N)');

subplot(3,1,2);
plot(zHat_cont(:,2), 'g');
title('Estimated Contact Force Y');
xlabel('Time Step');
ylabel('Force (N)');

subplot(3,1,3);
plot(zHat_cont(:,3), 'b');
title('Estimated Contact Force Z');
xlabel('Time Step');
ylabel('Force (N)');

% Plot the estimated contact torque components over time
figure;
subplot(3,1,1);
plot(zHat_cont(:,4), 'c');
title('Estimated Contact Torque X');
xlabel('Time Step');
ylabel('Torque (Nm)');

subplot(3,1,2);
plot(zHat_cont(:,5), 'm');
title('Estimated Contact Torque Y');
xlabel('Time Step');
ylabel('Torque (Nm)');

subplot(3,1,3);
plot(zHat_cont(:,6), 'k');
title('Estimated Contact Torque Z');
xlabel('Time Step');
ylabel('Torque (Nm)');


figure;

subplot(3,1,1);
plot(baseline_wrench.fx, 'r'); hold on;
title('Force (Baseline)');
legend({'baseline wrench Fx'}, 'Location', 'southwest');
hold off;

subplot(3,1,2);
plot(baseline_wrench.fy, 'g'); hold on;
title('Force (Baseline)');
legend({'baseline wrench Fy'}, 'Location', 'southwest');
hold off;

subplot(3,1,3);
plot(baseline_wrench.fz, 'b'); hold on;
title('Force (Baseline)');
legend({'baseline wrench Fz'}, 'Location', 'southwest');
hold off;

figure;

subplot(3,1,1);
plot(baseline_wrench.tx, 'c'); hold on;
title('Torque (Baseline)');
legend({'baseline wrench Tx'}, 'Location', 'southwest');
hold off;

subplot(3,1,2);
plot(baseline_wrench.ty, 'm'); hold on;
title('Torque (Baseline)');
legend({'baseline wrench Ty'}, 'Location', 'southwest');
hold off;

subplot(3,1,3);
plot(baseline_wrench.tz, 'k'); hold on;
title('Torque (Baseline)');
legend({'baseline wrench Tz'}, 'Location', 'southwest');
hold off;


figure;
subplot(3,1,1);
plot(X(:, 4)); title('Estimated Force in X ');
subplot(3,1,2);
plot(X(:, 5)); title('Estimated Force in Y ');
subplot(3,1,3);
plot(X(:, 6)); title('Estimated Force in Z ');
figure;
subplot(3,1,1);
plot(X(:, 7)); title('Estimated torque in X (Vibrations + Contact)');
subplot(3,1,2);
plot(X(:, 8)); title('Estimated torque in Y (Vibrations + Contact)');
subplot(3,1,3);
plot(X(:, 9)); title('Estimated torque in Z (Vibrations + Contact)');
