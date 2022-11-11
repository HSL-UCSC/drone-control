%% Clear workspace
close all;
clear;
clc;

%% Load IMU gyro/acc data
load IMUdata_handheld.mat;
datalength = size(gyrRec,1);


%% Filter gains
kP = 1;
kI = 0.1;
kW = 1; % Ki in paper for wMes (keep this 1, tune it last if necessary or just to see what happens)

%% Parameters
dt = 1/60;

%% States
% Estimated rotation matrices (body to inertial)
R_hat = [1 0 0;
        0 1 0;
        0 0 1];
eul = rotm2eul(R_hat,'XYZ');

% Estimated gryo bias
bias_hat = [0,0,0];

% Run 160Hz loop function -> ahrs_fusion_ag(&acc_ahrs, &gyro_ahrs, &ahrs);
for i=1:datalength
    % Get latest angular rate measurement
    omegaY = gyrRec(i,:)/1000000;
    
    % Store norm of acceleration MAYBE CAN JUST INSERT ON LINE 29
    accNorm = norm(accRec(i,:));

    %%% Might need to check if accNorm > 0 here %%%
    if(accNorm > 0)
        % Update v_a_hat 
        v_a_hat = R_hat(:,:,i)'*[0.0; 0.0; 1.0];
        % may want to normalize this v_a_hat as well ---------------- (rotation
        % matrix should maintain det=1
        
        % Update wMes --------- is this not being summed up throughout the entire length of data?
        wMes = kW*cross(accRec(i,:)/accNorm, v_a_hat); % Not positive if right (diff eq)
        
        % Update bDot
        biasDot = -kI*wMes;
        bias_hat(i+1,:) = bias_hat(i,:) + biasDot*dt;
    
        % Update RhatDot
        RhatDot = R_hat(:,:,i)*(skewSym(omegaY - bias_hat(i+1,:)) + kP*skewSym(wMes));
        R_hat(:,:,i+1) = R_hat(:,:,i) + RhatDot*dt;
        
        % Maybe normalize R_hat now (or the columns of R_hat)
        eul(i+1,:) = rotm2eul(R_hat(:,:,i+1),'XYZ');
    else
        R_hat(:,:,i+1) = R_hat(:,:,i);
        bias_hat(i+1,:) = bias_hat(i,:);
    end
end
R_hat(:,:,end) = [];
bias_hat(end,:) = [];
eul(end,:) = [];

figure()
plot(bias_hat)

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


disp(mean(Drone_attitude_data(:,1) - eul(:,1)))
disp(mean(Drone_attitude_data(:,2) - eul(:,2)))
% Define skew symmetric matrix
function skewSym=skewSym(vec)
    skewSym = [0 -vec(3) vec(2);
               vec(3) 0 -vec(1);
               -vec(2) vec(1) 0];
end

