%% Plot Full State
close all

%% 3D
% figure()
% plot3(FullState(:,5),FullState(:,6),FullState(:,7))
% grid on;
% axis equal


% hold on;
% for i=1:size(FullState,1)
%     plot3(FullState(1:i,5),FullState(1:i,6),FullState(1:i,7))
%     axis([0 0.5 0 1.0 0 1.0 ])
%     pause(0.01)
% end
% Plot Commanded Position, true position
figure()
hold on;
plot(FullState(:,1), FullState(:,2)) % cmd x
plot(FullState(:,1), FullState(:,5)) % mocap x
title("Commanded X Position")
legend("Command","MOCAP")
figure()
hold on;
plot(FullState(:,1), FullState(:,3)) % cmd x
plot(FullState(:,1), FullState(:,6)) % mocap x
title("Commanded Y Position")
legend("Command","MOCAP")
figure()
hold on;
plot(FullState(:,1), FullState(:,4)) % cmd x
plot(FullState(:,1), FullState(:,7)) % mocap x
title("Commanded Z Position")
legend("Command","MOCAP")

%%
% 
% % Position
% figure()
% subplot(3,1,1)
% hold on;
% plot(FullState(:,5))
% plot(FullState(:,2))
% title("X Trajectory")
% legend("X","Xref")
% 
% subplot(3,1,2)
% hold on;
% plot(FullState(:,6))
% plot(FullState(:,3))
% title("Y Trajectory")
% legend("Y","Yref")
% 
% subplot(3,1,3)
% hold on;
% plot(FullState(:,7))
% plot(FullState(:,4))
% title("Z Trajectory")
% legend("Z","Zref")
% 
% Attitude (I can also plot onboard attitude estimates vs the angle
% commands I sent)
figure()
subplot(3,1,1)
hold on;
plot(FullState(:,1), FullState(:,18)) %mocap
plot(FullState(:,1), FullState(:,15)) %ahrs
plot(FullState(:,1), FullState(:,12)) %cmd phi
% plot(trueCmdsRec(:,1))
plot(FullState(:,1), omegaD_REC(2,:)*180/pi)
title("Attitude Trajectory Tracking - ROLL")
legend("MOCAP","AHRS","Commanded Roll","Commanded Roll Rate")

subplot(3,1,2)
hold on;
plot(FullState(:,1), FullState(:,17)) %mocap
plot(FullState(:,1), FullState(:,14)) %ahrs
plot(FullState(:,1), FullState(:,11)) %cmd theta
% plot(trueCmdsRec(:,2))
plot(FullState(:,1), omegaD_REC(1,:)*180/pi)
title("Attitude Trajectory Tracking - PITCH")
legend("MOCAP","AHRS","Commanded Pitch","Commanded Pitch Rate")

subplot(3,1,3)
hold on;
plot(FullState(:,1), FullState(:,19)) %mocap
plot(FullState(:,1), FullState(:,16)) %ahrs
plot(FullState(:,1), FullState(:,13)) %cmd psi
plot(FullState(:,1), omegaD_REC(3,:)*180/pi)
title("Attitude Trajectory Tracking - YAW")
legend("MOCAP","AHRS","Commanded Yaw", "Commanded Yaw Rate")
% 

figure()
subplot(2,1,1)
hold on
plot(FullState(:,1), FullState(:,18)) %mocap
% plot(FullState(:,1), ahrsStRec(:,2)) %ST ahrs
plot(FullState(:,1), FullState(:,15)) %HSL ahrs
legend("MOCAP","ST","HSL")
title("Roll")

subplot(2,1,2)
hold on
plot(FullState(:,1), FullState(:,17)) %mocap
% plot(FullState(:,1), ahrsStRec(:,1)) %ST ahrs
plot(FullState(:,1), FullState(:,14)) %HSL ahrs
legend("MOCAP","ST","HSL")
title("Pitch")

%%
% PWM
figure()
hold on;
% plot(FullState(:,23))
% plot(FullState(:,24))
% plot(FullState(:,25))
plot(FullState(:,1), FullState(:,26))
title("PWM Signals")
legend("PWM 1", "PWM 2", "PWM 3", "PWM 4")

figure()
plot(loopTimes)
title("Loop Times")

figure()
plot(FullState(:,1), packetCount)
title("Packet Count")

% figure()
% plot(error_code)
% title("OE / PE")

figure()
hold on;
plot(FullState(:,1), torques(:,1))
plot(FullState(:,1), torques(:,2))
plot(FullState(:,1), torques(:,3))
title("Torques")

figure()
hold on;
plot(FullState(:,1), torques(:,4))
title(">3 error")

figure()
hold on;
plot(FullState(:,1), omegaD_REC(1,:))
plot(FullState(:,1), omegaD_REC(2,:))

figure()
plot(overrunFlag)

% figure()
% subplot(2,1,1);
% hold on;
% plot(omegaD_REC(1,:)*100)
% plot(FullState(:,11)) %cmd phi
% subplot(2,1,2);
% hold on;
% plot(omegaD_REC(2,:)*100)
% plot(FullState(:,12)) %cmd theta
%%
figure()
subplot(3,1,1)
hold on
plot(FullState(:,1),motorTorques(:,1))
% plot(FullState(:,1),FullState(:,15))
% legend("Roll Torque", "Roll")
title("Proportional")

subplot(3,1,2)
hold on
plot(FullState(:,1),motorTorques(:,2))
% plot(FullState(:,1),FullState(:,14))
% legend("Pitch Torque", "Pitch")
title("Integral")

subplot(3,1,3)
hold on
plot(FullState(:,1),motorTorques(:,3))
% plot(FullState(:,1),FullState(:,16))
% legend("Yaw Torque", "Yaw")
title("Derivative")
