%% Plot Full State
close all

%% 3D
figure()
plot3(FullState(:,5),FullState(:,6),FullState(:,7))
grid on;
axis equal
% hold on;
% for i=1:size(FullState,1)
%     plot3(FullState(1:i,5),FullState(1:i,6),FullState(1:i,7))
%     axis([0 0.5 0 1.0 0 1.0 ])
%     pause(0.01)
% end


%%

% Position
figure()
subplot(3,1,1)
hold on;
plot(FullState(:,5))
plot(FullState(:,2))
title("X Trajectory")
legend("X","Xref")

subplot(3,1,2)
hold on;
plot(FullState(:,6))
plot(FullState(:,3))
title("Y Trajectory")
legend("Y","Yref")

subplot(3,1,3)
hold on;
plot(FullState(:,7))
plot(FullState(:,4))
title("Z Trajectory")
legend("Z","Zref")

% Attitude (I can also plot onboard attitude estimates vs the angle
% commands I sent)
figure()
subplot(3,1,1)
hold on;
plot(FullState(:,18)) %mocap
plot(FullState(:,15)) %ahrs
plot(FullState(:,11)) %cmd phi
title("Attitude Trajectory Tracking - ROLL")
legend("MOCAP","AHRS","Commanded")

subplot(3,1,2)
hold on;
plot(-FullState(:,17)) %mocap
plot(-FullState(:,14)) %ahrs
plot(FullState(:,12)) %cmd theta
title("Attitude Trajectory Tracking - PITCH")
legend("MOCAP","AHRS","Commanded")

subplot(3,1,3)
hold on;
plot(FullState(:,19)) %mocap
plot(FullState(:,16)) %ahrs
plot(FullState(:,13)) %cmd psi
title("Attitude Trajectory Tracking - YAW")
legend("MOCAP","AHRS","Commanded")



% PWM
figure()
hold on;
plot(FullState(:,23))
plot(FullState(:,24))
plot(FullState(:,25))
plot(FullState(:,26))
title("PWM Signals")
legend("PWM 1", "PWM 2", "PWM 3", "PWM 4")

figure()
plot(loopTimes)
title("Loop Times")

figure()
plot(packetCount)
title("Packet Count")

figure()
plot(error_code)
title("OE / PE")

figure()
hold on;
plot(torques(:,1))
plot(torques(:,2))
title("Torques")
