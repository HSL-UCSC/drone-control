%% Clear workspace
close all;
clear;
clc;

%% Load IMU gyro/acc data
% load IMUdata_handheld.mat;
load IMUdata_autonomous6.mat; % 4 is manual override
datalength = size(gyrRec,1);


%% Filter gains
kP = 0.6; % For R estimate - Weight associated with the total error from sum of inertial cross prod. errors
kI = 0.01;  % For bias estimate - Weight associated with the total error from sum of inertial cross prod. errors
kW = 0.5; % Weights associated with how effective you treat each inertial direction error (ie. if you trust acc. inert. error more than magn. inert. error, then make gain higher for acc)

%% Parameters
dt = 1/60;
% radScaleFactor = 1000000;
radScaleFactor = 1/1.745329252e-5; % onboard
%% States
% Estimated rotation matrices (body to inertial)
R_hat = [1 0 0;
        0 1 0;
        0 0 1];
eul = rotm2eul(R_hat,'XYZ');

% Estimated gryo bias
bias_hat = [0,0,0];

dets = det(R_hat);
v_hats = [];
v_actual = [];

accNormRec = [];
AccThreshold = 20;

d = [0 0 0]';
c = 0.01;


% Run 160Hz loop function -> ahrs_fusion_ag(&acc_ahrs, &gyro_ahrs, &ahrs);
for i=1:datalength
    % Get latest angular rate measurement
    omegaY = gyrRec(i,:)/radScaleFactor;
    
    % Store norm of acceleration MAYBE CAN JUST INSERT ON LINE 29
    accNorm = norm(accRec(i,:));
    accNormRec(i) = accNorm;
    

    accIner = R_hat(:,:,i)*accRec(i,:)';
    
%     kP = 0.1*exp(0.1*abs(accNorm-980))
    %%% Might need to check if accNorm > 0 here %%%
    if(accNorm > 0)
        % Update v_a_hat 
        v_a_hat = R_hat(:,:,i)'*[0.0; 0.0; 1.0];
        
        % Plotting
        v_hats(:,i) = v_a_hat;
        v_actual(:,i) = accRec(i,:)'/accNorm;
        error(i) = sqrt((v_hats(1,i)-v_actual(1,i))^2 + (v_hats(2,i)-v_actual(2,i))^2 + (v_hats(3,i)-v_actual(3,i))^2);
    
%         Hard switching for accel dist rejection
        if(abs(accNorm - 980) > AccThreshold)
            kW = 1;
        else
            kW = 0.5;
        end

        % Soft switching for accel dist rejection
        kW = 0.5+exp(-abs(accNorm - 980)*0.001); % exp=2.718
%         kW = 0.5/(1+exp(abs(accNorm - 980)*0.001)); % Worse for some reason

%         d = d*c;
%         d = c*d;



        % Update wMes --------- is this not being summed up throughout the entire length of data?
        wMes = kW*cross(accRec(i,:)'/accNorm, v_a_hat);
        
        d = (accRec(i,:)'/accNorm - v_a_hat);

        % Update bDot
        biasDot = -kI.*wMes';
        bias_hat(i+1,:) = bias_hat(i,:) + biasDot*dt;
    
        % Update RhatDot
        RhatDot = R_hat(:,:,i)*(skewSym(omegaY - bias_hat(i+1,:)) + kP*skewSym(wMes));
        R_hat(:,:,i+1) = R_hat(:,:,i) + RhatDot*dt;
        
        % Normalize R_hat (should I do this?)
%         R_hat = R_hat(:,:,i) + RhatDot*dt;
%         [u s vt] = svd(R_hat);
%         R_hat(:,:,i+1) = u * vt';
        
    else
        R_hat(:,:,i+1) = R_hat(:,:,i);
        bias_hat(i+1,:) = bias_hat(i,:);
    end

    % Record
    eul(i+1,:) = rotm2eul(R_hat(:,:,i+1),'XYZ');
    dets(i+1) = det(R_hat(:,:,i+1));
end
R_hat(:,:,end) = [];
bias_hat(end,:) = [];
eul(end,:) = [];

% figure()
% plot(bias_hat)
% plot(dets)

figure()
hold on;
plot(Drone_attitude_data(:,1)*180/pi)
plot(eul(:,1)*180/pi)
title("Pitch Comparison")
legend("Truth","Estimated")
errorPitch = mean(abs(Drone_attitude_data(1:end,1)*180/pi - eul(:,1)*180/pi))

figure()
hold on;
plot(Drone_attitude_data(:,2)*180/pi)
plot(eul(:,2)*180/pi)
title("Roll Comparison")
legend("Truth","Estimated")
errorRoll = mean(abs(Drone_attitude_data(1:end,2)*180/pi - eul(:,2)*180/pi))

%%
% figure()
% plot(accNormRec-980)
% plot(accRec(:,1))
% hold on;
% plot(accRec(:,2))
% plot(accRec(:,3))

%%
% figure()
% q1=quiver3(0,0,0,v_hats(1,1),v_hats(2,1),v_hats(3,1))
% q2=quiver3(0,0,0,v_actual(1,1),v_actual(2,1),v_actual(3,1))
% for i=2:size(v_hats,2)
%     delete(q1)
%     delete(q2)
%     q1=quiver3(0,0,0,v_hats(1,i),v_hats(2,i),v_hats(3,i),"b") % The estimated inertial direction in body frame as understood with R_hat
%     q2=quiver3(0,0,0,v_actual(1,i),v_actual(2,i),v_actual(3,i),"r") % The current acceleration direction (at low frequency movement, this should match up to inertial direction in body)
%     % As the accelerometer moves, it stops representing the inertial
%     % direction, and thus, the error between the true inertial direction
%     % and our estimate grows
%     axis([-0.25 0.25 -0.25 0.25 0 1])
%     error(i)
%     hold on;
%     pause(0.01)
% end
% 
% figure()
% plot(error) % V_actual and V_hat should match up when the accelerometer isn't moving... R_Hat will be driven to the correct value, which will make V_hat and V_actual match up.
            % If acc. is moving, V_actual does not represent what V_hat is
            % estimating, however, it is still being used as an error to
            % drive R_Hat. This is an issue which means estimation doesn't
            % work well when the drone is moving translationally quick.
 