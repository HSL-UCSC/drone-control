%% Clear workspace
close all;
clear;
clc;

%% Load IMU gyro/acc data
% load IMUdata_handheld.mat;
load IMUdata_autonomous4.mat;
datalength = size(gyrRec,1);


%% Filter gains
kP = 0.6;
kI = 0.01;  %[0.05 0.01 1];
kW = 1; %[0.7 0.01 1]; % Ki in paper for wMes (keep this 1, tune it last if necessary or just to see what happens)

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

% Run 160Hz loop function -> ahrs_fusion_ag(&acc_ahrs, &gyro_ahrs, &ahrs);
for i=1:datalength
    % Get latest angular rate measurement
    omegaY = gyrRec(i,:)/radScaleFactor;
    
    % Store norm of acceleration MAYBE CAN JUST INSERT ON LINE 29
    accNorm = norm(accRec(i,:));

    %%% Might need to check if accNorm > 0 here %%%
    if(accNorm > 0)
        % Update v_a_hat 
        v_a_hat = R_hat(:,:,i)'*[0.0; 0.0; 1.0];
        % may want to normalize this v_a_hat as well ---------------- (rotation
        % matrix should maintain det=1
        
        % Update wMes --------- is this not being summed up throughout the entire length of data?
        wMes = kW.*cross(accRec(i,:)/accNorm, v_a_hat); % Not positive if right (diff eq)
        
        % Update bDot
        biasDot = -kI.*wMes;
        bias_hat(i+1,:) = bias_hat(i,:) + biasDot*dt;
    
        % Update RhatDot
        RhatDot = R_hat(:,:,i)*(skewSym(omegaY - bias_hat(i+1,:)) + kP*skewSym(wMes));
%         R_hat(:,:,i+1) = R_hat(:,:,i) + RhatDot*dt;
        
        % Normalize R_hat (should I do this?)
        R_hat = R_hat(:,:,i) + RhatDot*dt;
        [u s vt] = svd(R_hat);
        R_hat(:,:,i+1) = u * vt';
        
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

figure()
plot(bias_hat)
plot(dets)

figure()
hold on;
plot(Drone_attitude_data(:,1)*180/pi)
plot(eul(:,1)*180/pi)
title("Pitch Comparison")
legend("Truth","Estimated")

figure()
hold on;
plot(Drone_attitude_data(:,2)*180/pi)
plot(eul(:,2)*180/pi)
title("Roll Comparison")
legend("Truth","Estimated")


% disp(mean(Drone_attitude_data(:,1) - eul(:,1)*180/pi))
% disp(mean(Drone_attitude_data(:,2) - eul(:,2)*180/pi))


