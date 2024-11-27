%THis is for the Kalman classes
%Only one class definition is allowed per file for some reason
classdef KalmanFilter
    properties
        A % State transition matrix
        B % Input
        Dk % input
        H % Measurement
        Q % Covariance Process noise
        Rk % Measurement noise covariance
        P % Covariance of the state estimate
        
        biasForcTorque

        gaussianNoice

        % State estimate
        %initialized as state for the first iteration
        state 
        dataVariance
        XstateFull 
        XStateNew
        X
        zk
       
        % Initial state for
    end
    
    %The contact wrench (force and torque) between the robot and the environment can be estimated by subtracting
        %(1) the wrench arising from from gravity, (2) the robot motion, and (3) vibrations from the unbiased FTS
    %measurements.
    %Task: Once the biases have been estimated, they must be subtracted from all subsequent sensor measurement samples.
    %For the FTS samples, the bias must be subtracted from the samples prior to mass and mass center estimation.

    methods %ForceTorqueBias is [9.07633; -1.01814;  9.98482; 0.432449; -0.692162; -0.156746;]
        function obj = KalmanFilter(nStates, mass, massCenterScrewsym, ...
                frcTorquBias, massCenter, datasetAccel, datasetForceTorqu, gaussionNoiArray, varianceVec)
                        %Adding the two new ones, gaussiannoice and initial
                        
            
            %obj.XStateFull = zeros(stateDim, 9);
            


            %commented out 27.11.2024
            %datasetAccel.ax = datasetAccel.ax * -9.81;
            %datasetAccel.ay = datasetAccel.ay * -9.81;
            %datasetAccel.az = datasetAccel.az * -9.81;

            %varAccelAx = var(datasetAccel.ax); %defently the wrong data.
            %varAccelAy = var(datasetAccel.ay); % porbably baseline
            %varAccelAz = var(datasetAccel.az);

            obj.dataVariance = varianceVec;
            disp('This is the variance Vec:')
            disp(obj.dataVariance)



            obj.XStateNew = zeros(1, 9);
            %Initializing the state for the beginning
            obj.state = [datasetAccel.ax(1), datasetAccel.ay(1), datasetAccel.az(1),...
                datasetForceTorqu.fx(1), datasetForceTorqu.fy(1), datasetForceTorqu.fz(1),...
                datasetForceTorqu.tx(1), datasetForceTorqu.ty(1), datasetForceTorqu.tz(1)];
            disp('This is the state of')
            disp(obj.state)

            obj.A = eye(9);


            obj.gaussianNoice = gaussionNoiArray;

            
            obj.B = [eye(3); mass*eye(3); mass*massCenterScrewsym];
            
            obj.Dk = 0.1 * [zeros(3); zeros(3); zeros(3)];

            
            %For the report, we mostly care about the y and the z
            %force/torque
            tunedAccel = [1, 1/100, 1/1];

            tunedforce = [1, 1/10000, 1/500]; 

            tunedTorque = [1, 1/500, 1/1];



            
            obj.H = diag([tunedAccel, tunedforce, tunedTorque])
            
            obj.Q = [eye(3), zeros(3), zeros(3);
                    zeros(3), mass*eye(3), zeros(3);
                    zeros(3), zeros(3), mass*norm(massCenter)*eye(3)]; %9x9 matrix
            
            % = eye(measurement_dim);
            
            obj.P = eye(9);

            obj.biasForcTorque = frcTorquBias;
          


        end
        
        function obj = predict(obj, u, dt, currentStateVec)

            obj.zk = (obj.H* currentStateVec + (obj.Dk * u) + obj.gaussianNoice' .* (randn(9, 1)/1000))';
            obj.XStateNew = (obj.A * obj.state' + obj.B * u)';
            obj.P = obj.A * obj.P * obj.A' + dt*obj.Q;
            disp('output P')
            disp(obj.P)
            disp('OutputXStateNew')
            disp(obj.P)
        end
        
        function obj = update(obj)

            disp('input P')
            disp(obj.P)
            disp('input XStateNew')
            disp(obj.P)
            % Update step
            %Only place where Rk is used

            
            
            %Kalman gain 
            K = obj.P * obj.H' / (obj.H * obj.P * obj.H' + diag(obj.dataVariance)); % Kalman gain
                                                            %might have to
                                                            %do Rk
                                                            %differently

            

            %WHRONG Z is being used here.
        
            disp('This is the kalman filter stateBefore:')
            disp('This is the kalman filter state before:')
            disp(obj.XStateNew)
            %tuning by using the H matrix
            obj.state = (obj.XStateNew' + K * (obj.zk' - obj.H * obj.XStateNew'))';
            disp('This is the kalman filter state:')
            disp('This is the kalman filter state:')
            disp(obj.state)

            %updating covariance matrice for the next iteration
            obj.P = (eye(9) - K * obj.H) * obj.P;

            
        end
        
   
    end
end


