%THis is for the Kalman classes
%Only one class definition is allowed per file for some reason
classdef KalmanFilter
    properties
        A % State transition matrix
        B % Input
        Dk 
        H 
        Q 
        % Measurement noise covariance:
        Rk 
        % Covariance of the state estimate:
        P 
        
        biasForcTorque

        gaussianNoice
        processVariance
        % State estimate
        %initialized as state for the first iteration
        state 
        dataVariance
        process
        XstateFull 
        XStateNew
        X
        zk
        iterations
        iteration

        %lists for storing values for plotting/studying later
        kalmanGains
        covariances


      
       
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
                        
            
     

            obj.dataVariance = varianceVec;
            
            obj.iteration = 1;

            obj.iterations = length(datasetForceTorqu.fx);

            obj.kalmanGains = zeros(obj.iterations,1);
            obj.covariances = zeros(obj.iterations,1);

            obj.processVariance = 0.5;

            obj.XStateNew = zeros(1, 9);
            %Initializing the state for the first iteration
            obj.state = [datasetAccel.ax(1), datasetAccel.ay(1), datasetAccel.az(1),...
                datasetForceTorqu.fx(1), datasetForceTorqu.fy(1), datasetForceTorqu.fz(1),...
                datasetForceTorqu.tx(1), datasetForceTorqu.ty(1), datasetForceTorqu.tz(1)];
            

            obj.A = eye(9);


            obj.gaussianNoice = gaussionNoiArray;

            
            obj.B = [eye(3); mass*eye(3); mass*massCenterScrewsym];
            
            obj.Dk = [zeros(3); zeros(3); zeros(3)];

            
            %For the report, we mostly care about the y and the z
            %force/torque
            tunedAccel = [1/1, 1/100, 1/100];

            tunedforce = [-1, 1/20000, 1/10000]; 

            tunedTorque = [-1, 1/1, 1/1];



            
            obj.H = diag([tunedAccel, tunedforce, tunedTorque])
            
            obj.Q = [eye(3), zeros(3), zeros(3);
                    zeros(3), mass*eye(3), zeros(3);
                    zeros(3), zeros(3), mass*norm(massCenter)*eye(3)]; %9x9 matrix
            
            % = eye(measurement_dim);
            
            obj.P = eye(9);

            obj.biasForcTorque = frcTorquBias;
          


        end
        
        function obj = predict(obj, u, dt, currentStateVec)

            obj.zk = (obj.H* currentStateVec + (obj.Dk * u) +obj.gaussianNoice' .* randn(9, 1)/70)';
            obj.XStateNew = (obj.A * obj.state' + obj.B * u)';
            obj.P = obj.A * obj.P * obj.A' + dt*(obj.Q*obj.processVariance);
          
        end
        
        function obj = update(obj)

            

            % Update step
            %Only place where Rk is used

            
            
            %Kalman gain 
            K = obj.P * obj.H' / (obj.H * obj.P * obj.H' + diag(obj.dataVariance)); % Kalman gain
                                                            %might have to
                                                            %do Rk
                                                            %differently

            

            %WHRONG Z is being used here.
        
          
            %tuning by using the H matrix
            obj.state = (obj.XStateNew' + K * (obj.zk' - obj.H * obj.XStateNew'))';
          

            %updating covariance matrice for the next iteration
            obj.P = (eye(9) - K * obj.H) * obj.P;
      
            
            isDiagonal = isequal(obj.P, diag(diag(obj.P)));

            if isDiagonal
                
            else
                disp('The matrix is not diagonal.');
            end

            obj.kalmanGains(obj.iteration) = det(K);
            obj.covariances(obj.iteration) = det(obj.P);

            obj.iteration = obj.iteration + 1;

            
        end

        function obj = plotKnQ(obj)

            

            figure;
            %plot((obj.zHatAll(:, 3) - obj.forceBias(3) - obj.Vg(3)), 'b-', 'DisplayName', 'z^3'); % Plot z^3
            plot((obj.covariances), 'b-', 'DisplayName', 'det(Q_k)'); % Plot z^3
            hold on;
            xline(1000, '--r', 'DisplayName', 'Iteration  = 1000');
            hold off;
            title('Determinant of covariance matices');
            xlabel('Time Step');
            ylabel('Value');
            legend('show'); % Show legend
            grid on;
            
            
            figure;
            %plot((obj.zHatAll(:, 5) - obj.torqueBias(2) - obj.Vg(5)), 'b-', 'DisplayName', 'z^5');
            plot((obj.kalmanGains), 'b-', 'DisplayName', 'k_k');
            hold on;
            xline(1000, '--r', 'DisplayName', 'Iteration  = 1000');
            hold off;
            title('Kalman gains per iteration');
            xlabel('Iteration');
            ylabel('Value');
            legend('show'); % Show legend
            grid on;


            
        end

        
        
   
    end
end


