%% Clear workspace
close all;
clear;
clc;

%% Load IMU gyro/acc data
load imuData;
datalength = size(gyroMeas,1);


%% Filter gains
Kp = 1;
kI = 1;
kW = 1; % Ki in paper for wMes (keep this 1, tune it last if necessary or just to see what happens)

%% Parameters
dt = 1/160;

%% States
% Estimated rotation matrices (body to inertial)
R_hat = [1 0 0;
        0 1 0;
        0 0 1];

% Estimated gryo bias
bias_hat = [0,0,0];

% Run 160Hz loop function -> ahrs_fusion_ag(&acc_ahrs, &gyro_ahrs, &ahrs);
for i=i:datalength
    % Get latest angular rate measurement
    omegaY = gyro(i,:);
    
    % Store norm of acceleration MAYBE CAN JUST INSERT ON LINE 29
    accNorm = norm(acc(i,:));

    %%% Might need to check if accNorm > 0 here %%%
    % Update v_a_hat 
    v_a_hat = R_hat(:,:,i).T*[0.0, 0.0, 1.0];
    % may want to normalize this v_a_hat as well ---------------- (rotation
    % matrix should maintain det=1
    
    % Update wMes --------- is this not being summed up throughout the entire length of data?
    wMes = kW*cross(acc(i,:)/accNorm, v_a_hat); % Not positive if right (diff eq)
    
    % Update bDot
    biasDot = -kI*wMes;
    bias_hat(i+1,:) = bias_hat(i,:) + biasDot*dt;

    % Update RhatDot
    RhatDot = R_hat(:,:,i)*(skewSym(omegaY - bias_hat(i+1,:)) + kP*skewSym(wMes));
    R_hat(:,:,i+1) = R_hat(:,:,i) + RhatDot*dt;

    % Maybe normalize R_hat now (or the columns of R_hat)
end


% Define skew symmetric matrix
function skewSym=skewSym(vec)
    skewSym = [0 -vec(3) vec(2);
               vec(3) 0 -vec(1);
               -vec(2) vec(1) 0];
end

