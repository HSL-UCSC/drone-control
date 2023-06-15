%% Plot Full State
close all


% Plot Time
figure()
plot(diff(FullState(:,1)))
title("Loop Times")

% Packet count
figure()
plot(packetCount);


% Plot Commanded Position, true position
figure()
hold on;
plot(FullState(:,2)) % cmd x
plot(FullState(:,5)) % mocap x
title("Commanded X Position")
legend("Command","MOCAP")
figure()
hold on;
plot(FullState(:,3)) % cmd x
plot(FullState(:,6)) % mocap x
title("Commanded Y Position")
legend("Command","MOCAP")
figure()
hold on;
plot(FullState(:,4)) % cmd x
plot(FullState(:,7)) % mocap x
title("Commanded Z Position")
legend("Command","MOCAP")


% Plot Velocities
figure()
hold on;
plot(FullState(:,8))
plot(FullState(:,9))
plot(FullState(:,10))
title("Velocities")

% Plot commanded thrusts
figure()
plot(FullState(:,14));
title("Commanded Thrust")


% Plot cmdAttitude,attitude,mocapAttitude
figure()
subplot(3,1,1)
hold on;
plot(-FullState(:,11)) %  cmd pitch
plot(FullState(:,15))
plot(FullState(:,39))
title("Commanded Pitch")
legend("Command","AHRS", "MOCAP")

subplot(3,1,2)
hold on;
plot(FullState(:,12)) %  cmd roll
plot(FullState(:,16))
plot(FullState(:,40))
title("Commanded Roll")
legend("Command","AHRS", "MOCAP")

subplot(3,1,3)
hold on;
plot(FullState(:,13)) %  cmd yaw
plot(FullState(:,17))
plot(FullState(:,41))
title("Commanded Yaw")
legend("Command","AHRS", "MOCAP")


% Plot Attitude Rate
figure()
hold on;
plot(FullState(:,18))
plot(FullState(:,42),'m--')
title("Pitch Rate")
legend("AHRS", "MOCAP")
figure()
hold on;
plot(FullState(:,19))
plot(FullState(:,43),'m--')
title("Roll Rate")
legend("AHRS", "MOCAP")
figure()
hold on;
plot(FullState(:,20))
plot(FullState(:,44),'m--')
title("Yaw Rate")
legend("AHRS", "MOCAP")



% Plot attitude rate error
figure()
hold on;
plot(FullState(:,21))
plot(FullState(:,22))
plot(FullState(:,23))
title("Attitude Rate Error")
figure()
hold on;
plot(FullState(:,24))
plot(FullState(:,25))
plot(FullState(:,26))
title("Attitude Rate Error Derivatives")
figure()
hold on;
plot(FullState(:,27))
plot(FullState(:,28))
plot(FullState(:,29))
title("Attitude Rate Error Integrals")




% Plot Position error
figure()
hold on;
plot(FullState(:,30))
plot(FullState(:,31))
plot(FullState(:,32))
title("Position Error")

figure()
hold on;
plot(FullState(:,33))
plot(FullState(:,34))
plot(FullState(:,35))
title("Position Error Derivatives")

figure()
hold on;
plot(FullState(:,36))
plot(FullState(:,37))
plot(FullState(:,38))
title("Position Error Integrals")


