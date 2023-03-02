%% Plot Full State
close all

% Position
figure()
subplot(3,1,1)
hold on;
plot(FullState(:,1))
plot(xRefs)
title("X Trajectory")
subplot(3,1,2)
hold on;
plot(FullState(:,2))
plot(yRefs)
title("Y Trajectory")
subplot(3,1,3)
hold on;
plot(FullState(:,3))
plot(zRefs)
title("Z Trajectory")

% Attitude (I can also plot onboard attitude estimates vs the angle
% commands I sent)
figure()
subplot(3,1,1)
hold on;
plot(FullState(:,7)*180/pi)
plot(ahrsRec(:,1))
subplot(3,1,2)
hold on;
plot(FullState(:,8)*180/pi)
plot(ahrsRec(:,2))
subplot(3,1,3)
hold on;
plot(FullState(:,9)*180/pi)
plot(ahrsRec(:,3))
title("Attitude Trajectory")

% PWM
figure()
hold on;
plot(FullState(:,13))
plot(FullState(:,14))
plot(FullState(:,15))
plot(FullState(:,16))
title("PWM Signals")
legend("PWM 1", "PWM 2", "PWM 3", "PWM 4")

figure()
plot(loopTimes)

figure()
plot(packetCount)

