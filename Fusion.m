
classdef Fusion
    properties
        kalmanFilter % Kalman Instance
        mass 
        mass_center % Mass center of the system
        gravityVec % Gravity vector
        Rfs % Rotation matrix from IMU to FTS frame
        

        %This is Biases
        Vg
        forceBias
        torqueBias

        
        
        % measurement matrices
        H_a 
        H_f
        H_c 

        %sampling rates
        fr
        fa
        ff

        %wrenchDataset

        zHatAll
        xHatAll

    end
    
    methods
        function obj = Fusion(kalmanFilter)
            % Constructor initializes the Fusion object
            obj.kalmanFilter = kalmanFilter;
        end
        
        
        
        function obj = setupStateSpace(obj, mass, mass_center, Rfs, gravityVec, n, forceBias, torqueBias, Vg)
            % Setup state-space model
            obj.mass = mass;
            obj.mass_center = mass_center;
            obj.gravityVec = gravityVec;
            obj.Rfs = Rfs;
            
            obj.H_a = [eye(3), zeros(3, 6)];
            obj.H_f = [zeros(3), eye(3), zeros(3);
                       zeros(3, 6), eye(3)];
            
            mass_center_skew = [0, -mass_center(3), mass_center(2);
                                mass_center(3), 0, -mass_center(1);
                                -mass_center(2), mass_center(1), 0];
            
            obj.H_c = [-mass * eye(3), eye(3), zeros(3);
                       -mass * mass_center_skew, zeros(3), eye(3)];
            
            obj.fr = 100.2;
            obj.fa = 254.3;
            obj.ff = 698.3;

            obj.zHatAll = zeros(n, 6); 
            obj.xHatAll = zeros(n, 9); 

            obj.forceBias = forceBias; 
            obj.torqueBias = torqueBias; 
            obj.Vg = Vg;


            

        end
        
        function run(obj, wrenchDataset, accelDataset, orien, lastTrajectory)
            n = height(wrenchDataset);

            orientationsDatasetIndex = 1;
            accelDatasetIndex = 1; 


            ax = accelDataset.ax(accelDatasetIndex);
            ay = accelDataset.ay(accelDatasetIndex);
            az = accelDataset.az(accelDatasetIndex);

            lastTrajectory = lastTrajectory*obj.gravityVec;


            orentationsDataLength = length(orien.r11)- 1;
            accelDataLength = length(accelDataset.ax) -1;


            deltaG_sk = ([orien.r11(orientationsDatasetIndex), orien.r12(orientationsDatasetIndex), orien.r13(orientationsDatasetIndex); orien.r21(orientationsDatasetIndex), orien.r22(orientationsDatasetIndex), orien.r23(orientationsDatasetIndex); orien.r31(orientationsDatasetIndex), orien.r32(orientationsDatasetIndex), orien.r33(orientationsDatasetIndex)]*obj.gravityVec - lastTrajectory);
            disp("deltaG_sk")
            disp(deltaG_sk)
            

            obj.Rfs = [orien.r11(orientationsDatasetIndex), orien.r12(orientationsDatasetIndex), orien.r13(orientationsDatasetIndex);
                     orien.r21(orientationsDatasetIndex), orien.r22(orientationsDatasetIndex), orien.r23(orientationsDatasetIndex);
                     orien.r31(orientationsDatasetIndex), orien.r32(orientationsDatasetIndex), orien.r33(orientationsDatasetIndex)];
            u_k = deltaG_sk * (obj.fr / (obj.ff + obj.fa));
            disp("u_k:")
            disp(u_k)
            
            
            disp(obj.gravityVec)
            disp("obj.gravityVec")
            disp("obj.gravityVec")

            %obj.wrenchDataset = wrenchDataset;

            for i = 2:n

                while (wrenchDataset.t(i) >  orien.t(orientationsDatasetIndex)) & (orientationsDatasetIndex < orentationsDataLength)
  
                deltaG_sk = ([orien.r11(orientationsDatasetIndex), orien.r12(orientationsDatasetIndex), orien.r13(orientationsDatasetIndex); orien.r21(orientationsDatasetIndex), orien.r22(orientationsDatasetIndex), orien.r23(orientationsDatasetIndex); orien.r31(orientationsDatasetIndex), orien.r32(orientationsDatasetIndex), orien.r33(orientationsDatasetIndex)]*obj.gravityVec - lastTrajectory);
       

                 %Orientation (Rfs):
                 obj.Rfs = [orien.r11(orientationsDatasetIndex), orien.r12(orientationsDatasetIndex), orien.r13(orientationsDatasetIndex);
                     orien.r21(orientationsDatasetIndex), orien.r22(orientationsDatasetIndex), orien.r23(orientationsDatasetIndex);
                     orien.r31(orientationsDatasetIndex), orien.r32(orientationsDatasetIndex), orien.r33(orientationsDatasetIndex)];
                 u_k = deltaG_sk * (obj.fr / (obj.ff + obj.fa));
                
  
                   lastTrajectory = [orien.r11(orientationsDatasetIndex), orien.r12(orientationsDatasetIndex), orien.r13(orientationsDatasetIndex);
                       orien.r21(orientationsDatasetIndex), orien.r22(orientationsDatasetIndex), orien.r23(orientationsDatasetIndex);
                       orien.r31(orientationsDatasetIndex), orien.r32(orientationsDatasetIndex), orien.r33(orientationsDatasetIndex)]*obj.gravityVec;
                  orientationsDatasetIndex = orientationsDatasetIndex + 1;  
                    
                 end 
            
               while (wrenchDataset.t(i) >  accelDataset.t(accelDatasetIndex)) & (accelDatasetIndex < accelDataLength)
                    
                    accelDatasetIndex = accelDatasetIndex + 1; 
                    %Already did multiply with 9.81
                    ax = accelDataset.ax(accelDatasetIndex);%*-9.81;
                    ay = accelDataset.ay(accelDatasetIndex);%*-9.81;
                    az = accelDataset.az(accelDatasetIndex);%*- 9.81;
            
                    %accelVec = [ax; ay; az];
                end 
            
                           


        
                
               %wrenchVec = [wrench.fx(i); wrench.fy(i); wrench.fz(i); wrench.tx(i); wrench.ty(i); wrench.tz(i)];
                
                
                
                % Kalman Filter Prediction:
                %Using dynamic dt
                dt = wrenchDataset.t(i) - wrenchDataset.t(i-1); 


                currentStateVec = [[ax; ay; az]*9.81;...
                    wrenchDataset.fx(i); wrenchDataset.fy(i); wrenchDataset.fz(i); ...
                    wrenchDataset.tx(i); wrenchDataset.ty(i); wrenchDataset.tz(i)];

                obj.kalmanFilter = obj.kalmanFilter.predict(u_k, dt, currentStateVec);
                
                %zk = (H_k*[ax,ay,az,X_f_t]' + obj.Dk*u_k + gaussian_noise)';
                

                

                

                % Kalman Filter Update
                obj.kalmanFilter = obj.kalmanFilter.update();
                

                currentState = obj.kalmanFilter.state;
             

                
                %z_hat_k = (obj.H_c * [[ax; ay; az]; ...
                %    wrenchDataset.fx(i); wrenchDataset.fy(i); wrenchDataset.fz(i); ...
                %    wrenchDataset.tx(i); wrenchDataset.ty(i); wrenchDataset.tz(i)]);
                


                z_hat_k = (obj.H_c * (diag([[1, 1, 1], [1, 1, 1], [1, 1, 1]])*(currentState')));%/9.81));
                %[[ax; ay; az]; ...
                %    wrenchDataset.fx(i); wrenchDataset.fy(i); wrenchDataset.fz(i); ...
                %    wrenchDataset.tx(i); wrenchDataset.ty(i); wrenchDataset.tz(i)]);
               
            
                disp('z_hat_k')
                disp('z_hat_k')
                disp(z_hat_k)

                obj.zHatAll(i, :) =  z_hat_k';

                disp('obj.kalmanFilter.state')
                disp('Collected this cockhead obj.kalmanFilter.state')
                disp('obj.kalmanFilter.state')

                obj.xHatAll(i, :) = obj.kalmanFilter.state'; 
                obj.xHatAll(i, :) = currentState'; 


                % Output estimated state
                %disp(['Step ', num2str(i), ': Estimated State = ', num2str(obj.kalmanFilter.state')]);
            end
            

            
        
            %Calling the plotting function
            plot(obj,wrenchDataset)



        end
        function plot(obj, wrenchDataset)
            % Plot z^3, x_6, and Force f_3, ensuring xHatAll is plotted last
            figure;
            plot((obj.zHatAll(:, 3) - obj.forceBias(3) - obj.Vg(3)), 'b-', 'DisplayName', 'z^3'); % Plot z^3
            hold on;
            plot((wrenchDataset.fz - obj.forceBias(3) - obj.Vg(3)), 'g-.', 'DisplayName', 'Force f_3'); 
            plot((obj.xHatAll(:, 6) - obj.forceBias(3) - obj.Vg(3)), 'r--', 'LineWidth', 1.2, 'DisplayName', 'x_6');
            hold off;
            title('z^3, x_6, and Force f_3');
            xlabel('Time Step');
            ylabel('Value');
            legend('show'); % Show legend
            grid on;
            
            
            figure;
            %plot((obj.zHatAll(:, 5) - obj.torqueBias(2) - obj.Vg(5)), 'b-', 'DisplayName', 'z^5');
            plot((obj.zHatAll(:, 5) - obj.Vg(5)), 'b-', 'DisplayName', 'z^5');
            hold on;
            plot((wrenchDataset.ty - obj.torqueBias(2) - obj.Vg(5)), 'g-.', 'DisplayName', 'Torque t_2'); 
            plot((obj.xHatAll(:, 8) - obj.torqueBias(2) - obj.Vg(5)), 'r--', 'LineWidth', 1.2, 'DisplayName', 'x_8');
            hold off;
            title('z^5, x_8, and Torque t_2');
            xlabel('Iteration');
            ylabel('Value');
            legend('show'); % Show legend
            grid on;


        end
    end
end
