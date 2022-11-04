close all

figure()
plot(Drone_data(:,5)*180/pi)
% hold on;
% plot(desired_attitudes(:,1))
title("Pitch");

figure()
plot(Drone_data(:,6)*180/pi)
% hold on;
% plot(desired_attitudes(:,2))
title("Roll");


figure()
plot(p_x,p_y)
axis([-2 2 -2 2])

% PID Output X
figure()
plot(cont_actual_data(:,1:3))
title("PID X Output");
legend("p","i","d")

% PID Output Y
figure()
plot(cont_actual_data(:,4:6))
title("PID Y Output");
legend("p","i","d")

% PID Output Z
figure()
plot(cont_actual_data(:,7:9))
title("PID Z Output");
legend("p","i","d")
% 
% 
figure()
plot(sent_data)
title("Commands Sent");
legend("comm_thr", "comm_phi", "comm_theta")
% 
% figure()
% plot(errors)

figure()
plot(loopTimes)
title("Loop time")

figure()
plot(packetCount)

% Find timeshift (and corresponding time delay) through xcorr (time delay should be 10-15ms (ie. = read() time?))
% [c,lags] = xcorr(euler(:,1),Drone_pos_data(:,1)*(180/pi));
% timeShift = find(c==max(c)) - k; % This number corresponds to how many ms?
% timeShift = 0;
% RMSE_pitch_angle = sqrt(mean((euler(1+timeShift:end,1) - Drone_pos_data(1:end-timeShift,1)*(180/pi)).^2))  % Root Mean Squared Error
% % Plot with timeshift
% figure();
% plot(euler(1+timeShift:end,1));
% hold on;
% plot(Drone_pos_data(1:end-timeShift,1)*(180/pi))
legend("drone thx-pitch","motive thx-pitch");
figure();
plot(euler(1:end,1));
hold on;
plot(Drone_pos_data(1:end,1)*(180/pi))
legend("drone thx-pitch","motive thx-pitch");


% Find timeshift
% [c,lags] = xcorr(euler(:,2),Drone_pos_data(:,2)*(180/pi));
% timeShift = find(c==max(c)) - k; % This number corresponds to how many ms?
% timeShift = 0;
% RMSE_roll_angle = sqrt(mean((euler(1+timeShift:end,2) - Drone_pos_data(1:end-timeShift,2)*(180/pi)).^2))  % Root Mean Squared Error
% % Plot with timeshift
% figure();   
% plot(euler(1+timeShift:end,2));
% hold on;
% plot(Drone_pos_data(1:end-timeShift,2)*180/pi)
legend("drone thy-roll","motive thy-roll");
figure();   
plot(euler(1:end,2));
hold on;
plot(Drone_pos_data(1:end,2)*180/pi)
legend("drone thy-roll","motive thy-roll");



pitchRatesFil_motive = lowpass(euler_rates(:,1)*180/pi,0.1);
pitchRatesFil_drone = lowpass(Drone_rate_data(:,1)*180/pi,0.1);
figure();
plot(pitchRatesFil_motive);
hold on;
plot(pitchRatesFil_drone)
legend("drone thx-pitch rate","motive thx-pitch rate");

rollRatesFil_motive = lowpass(euler_rates(:,2)*180/pi,0.1);
rollRatesFil_drone = lowpass(Drone_rate_data(:,2)*180/pi,0.1);
figure();
plot(rollRatesFil_motive);
hold on;
plot(rollRatesFil_drone)
legend("drone thy-roll rate","motive thy-roll rate");




    
%% Attitude tracking
close all
figure()
hold on;
plot(p_z)
plot(z_ref_final*ones(size(p_z,1)));
title("Height");
legend("Z", "Z_ref")
axis([0 k 0 2])
grid on;

figure()
hold on;
plot(p_x)
plot(xRefs);
title("X");
legend("X", "X_ref")
axis([0 k -2 2])

figure()
hold on;
plot(p_y)
plot(yRefs);
title("Y");
legend("Y", "Y_ref")
axis([0 k -2 2])

% 
% 
% figure()
% plot(sent_data)
% title("Commands Sent");
% legend("comm_thr", "comm_phi", "comm_theta")
% 


%% Tyler
% figure()
% plot(Tyler(:,1))
% 
% figure()
% plot(Tyler(:,2))
% 
% figure()
% plot(Tyler(:,3))
% 
% figure()
% plot(Tyler(:,16))

%% Tracking
% close all


% figure()
% plot(loopTimes)
% title("Loop time")
% 
% figure()
% plot(packetCount)
% title("Packet count")
% 
% figure()
% hold on;
% % plot(-desired_attitudes(:,1));
% plot(pwmSignals(:,1))
% plot(Drone_pos_data(:,1)*180/pi)
% plot(ahrsRec(:,1))
% title("Pitch tracking")
% legend("Commanded","MOCAP","AHRS")
% 
% figure()
% hold on;
% % plot(desired_attitudes(:,2));
% plot(pwmSignals(:,2))
% plot(Drone_pos_data(:,2)*180/pi)
% plot(ahrsRec(:,2))
% title("Roll tracking")
% 
% legend("Commanded","MOCAP","AHRS")



% figure()
hold on;
% plot(Drone_pos_data(1:end-1,1)*180/pi - ahrsRec(:,1))
% title("Pitch estimation error")

% figure()
plot(Drone_pos_data(1:end-1,2)*180/pi - ahrsRec(:,2))
title("Roll estimation error")


