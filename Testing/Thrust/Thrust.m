%% Clear past data
close all;  
clear all;
clc;

%% Instantiate client object to run Motive API commands
% https://optitrack.com/software/natnet-sdk/

% Create Motive client object
dllPath = fullfile('d:','StDroneControl','NatNetSDK','lib','x64','NatNetML.dll');
assemblyInfo = NET.addAssembly(dllPath); % Add API function calls
theClient = NatNetML.NatNetClientML(0);

% Create connection to localhost, data is now being streamed through client object
HostIP = '127.0.0.1';
theClient.Initialize(HostIP, HostIP); 


%% Connect to the Drone via Bluetooth
%b = ble("C0283C361730");
b = ble("C0286E324A33"); % Make sure to get the the Drone's MAC address before running this code
char = b.Characteristics; % Get the characteristics of the Drone

%% Assign the rigid body id. Double check with motive that the rigid body ID is correct.killswitch

Drone_ID = 1;

%% Store the reference to the Charactersitic responsible for Writing Joystick data:
% https://www.st.com/resource/en/user_manual/dm00550659-getting-started-with-the-bluest-protocol-and-sdk-stmicroelectronics.pdf
        % To send Joydata you need to send an array of 7 elements:
        % First element:- No idea what this does.
        % Second element:- rudder value (gRUD) 128 is like sending 0 (YAW)
        % Third element:- thrust value (gTHR) (THRUST)
        % Fourth element:- AIL value (gAIL) 128 is like sending 0  (ROLL)
        % Fifth element:- ELE value (gELE) 128 is like sending 0  (PITCH)
        % Sixth element:- SEEKBAR value: mainly from android app just send 0.
        % Seventh element:- ARMING and CALIB: 0x04, 0x05, 0x01(For calib)
joy_c = characteristic(b, "00000000-0001-11E1-9AB4-0002A5D5C51B" , "00008000-0001-11E1-AC36-0002A5D5C51B") % Write w/out response
% joy_c_imu = characteristic(b, "00000000-0001-11E1-9AB4-0002A5D5C51B" , "00E00000-0001-11E1-AC36-0002A5D5C51B") % Read IMU


%% Next calibrate/arm the drone
% Reference slides 25,26 from - https://www.st.com/content/ccc/resource/sales_and_marketing/presentation/product_presentation/group0/bd/cc/11/15/14/d4/4a/85/STEVAL-DRONE01_GETTING_STARTED_GUIDE/files/steval-drone01_getting_started_guide.pdf/jcr:content/translations/en.steval-drone01_getting_started_guide.pdf

% 1) Place drone down flat and press reset button to calibrate it
% 2) Arm the drone: 
write(joy_c, [0, 0, 0, 0, 0, 0, 2], 'uint8', "WithoutResponse") % is code running on drone different? Should send a 4 to arm?
java.lang.Thread.sleep(2*1000); % Java sleep is much more accurate than matlab's pause (sleep in ms)
write(joy_c, [22, 128, 0, 128, 128, 0, 0], 'uint8', "WithoutResponse");

%% Set up data collection vectors
ITERATIONS = 1050;
WARMUP = 100;

% Data collection vectors
Drone_data = [];  % drone position data
sent_data = []; % data sent to drone

% X,Y,Z position data
p_x = [];
p_y = [];
p_z = [];

% X,Y,Z position data (non-filtered)
nf_x = [];
nf_y= [];
nf_z = [];

% X,Y,Z velocity data 
f_v_x = [];
f_v_y = [];
f_v_z = [];

% X,Y,Z velocity data (non-filtered)
nf_v_x = [];
nf_v_y = [];
nf_v_z = [];

%% Frequencies
% What are these used for ?
OUT_FREQ = 60;
CUT_OFF_FREQ_POS = 10;
CUT_OFF_FREQ_VEL = 10;

%% Mass of the drone
m = 86.55/1000; % 86.55g
MAX_ANGLE = 30; % 30 deg

%% Inititalize the PID controllers -------------------------------------------------
X_pid = VXpid_error_init(1, OUT_FREQ, CUT_OFF_FREQ_VEL);
Y_pid = VYpid_error_init(1, OUT_FREQ, CUT_OFF_FREQ_VEL);
Z_pid = VZpid_error_init(1, OUT_FREQ, CUT_OFF_FREQ_VEL);

%% Get the Drone data
[DronePos] = GetDronePosition(theClient, Drone_ID);
Drone_data(1, :) = DronePos;

%% SET POINT TO TRACK
x_ref = 0.0;
y_ref = 0.0;
z_ref = 0.0;
psi = 0;
phi_d = 0;
theta_d = 0;
%% Initialize lowpass filter
[DronePos] = GetDronePosition(theClient, Drone_ID);
lpfData_x = lpf_2_init(OUT_FREQ, CUT_OFF_FREQ_POS, DronePos(2));
lpfData_y = lpf_2_init(OUT_FREQ, CUT_OFF_FREQ_POS, DronePos(3));
lpfData_z = lpf_2_init(OUT_FREQ, CUT_OFF_FREQ_POS, DronePos(4));

lpfData_vx = lpf_2_init(OUT_FREQ, CUT_OFF_FREQ_VEL, 0);
lpfData_vy = lpf_2_init(OUT_FREQ, CUT_OFF_FREQ_VEL, 0);
lpfData_vz = lpf_2_init(OUT_FREQ, CUT_OFF_FREQ_VEL, 0);

prev_x =  DronePos(2);
prev_y =  DronePos(3);
prev_z =  DronePos(4);
prev_nf_x =  DronePos(2);
prev_nf_y =  DronePos(3);
prev_nf_z =  DronePos(4);

%% Warm up Filter
for i = 1:100
    [DronePos] = GetDronePosition(theClient, Drone_ID);
    %Drone_data = [Drone_data; DronePos];
    
    [x_f, lpfData_x] = lpf_2(lpfData_x, DronePos(2));
    [vx_f, lpfData_vx] = lpf_2(lpfData_vx, x_f - prev_x);
    prev_x = x_f;
    
    [y_f, lpfData_y] = lpf_2(lpfData_y, DronePos(3));
    [vy_f, lpfData_vy] = lpf_2(lpfData_vy, y_f - prev_y);
    prev_y = y_f;
    
    [z_f, lpfData_z] = lpf_2(lpfData_z, DronePos(4));
    [vz_f, lpfData_vz] = lpf_2(lpfData_vz, z_f - prev_z);
    prev_z = z_f;
    
end

%% Test thrust
% Sleep 10 seconds
java.lang.Thread.sleep(10*1000);
disp("starting")

% Command a slowly rising thrust
thrust = 0;
for k=1:165
    % Command a new thrust
    thrust = thrust + 1
    java.lang.Thread.sleep(100); % 10 seconds to go from 0-100
    write(joy_c, [0, 128, thrust, 128, 128, 0, 5], 'uint8', "WithoutResponse")
    
    
    
    % Get new drone position, apply filter, store it and velocity estimate
    [DronePos] = GetDronePosition(theClient, Drone_ID);
    Drone_data(k, :) = DronePos;
    
    
    
    % Capture non-filtered measurements
    nf_x(k) = DronePos(2);
    nf_y(k) = DronePos(3);
    nf_z(k) = DronePos(4);
    
    nf_v_x(k) = (nf_x(k) - prev_nf_x)/(1/120); % store non-filtered velocity
    prev_nf_x = nf_x(k); % Capture previous non-filtered position

    nf_v_y(k) = (nf_y(k) - prev_nf_y)/(1/120); % store non-filtered velocity
    prev_nf_y = nf_y(k); % Capture previous non-filtered position
    
    nf_v_z(k) = (nf_z(k) - prev_nf_z)/(1/120); % store non-filtered velocity
    prev_nf_z = nf_z(k); % Capture previous non-filtered position
    
    
    
    % Apply low pass filter to position measurements
    [x_f, lpfData_x] = lpf_2(lpfData_x, DronePos(2));
    p_x(k) = x_f;
    [vx_f, lpfData_vx] = lpf_2(lpfData_vx, (x_f - prev_x)/(1/120));
    f_v_x(k) = vx_f;
    prev_x = x_f;
    
    [y_f, lpfData_y] = lpf_2(lpfData_y, DronePos(3));
    p_y(k) = y_f;
    [vy_f, lpfData_vy] = lpf_2(lpfData_vy, (y_f - prev_y)/(1/120));
    f_v_y(k) = vy_f;
    prev_y = y_f;
    
    [z_f, lpfData_z] = lpf_2(lpfData_z, DronePos(4));
    p_z(k) = z_f;
    [vz_f, lpfData_vz] = lpf_2(lpfData_vz, (z_f - prev_z)/(1/120));
    f_v_z(k) = vz_f;
    prev_z = z_f;
end

%Stay at max speed for 10 seconds
disp("Max speed reached, staying there for 10 seconds");
java.lang.Thread.sleep(10000);


% Command thrust down to zero
for k=1:165
    % Command a new thrust
    thrust = thrust - 1
    java.lang.Thread.sleep(100); % 20 seconds to go from 0-200
    write(joy_c, [0, 128, thrust, 128, 128, 0, 5], 'uint8', "WithoutResponse")
    
    
    
    % Get new drone position, apply filter, store it and velocity estimate
    [DronePos] = GetDronePosition(theClient, Drone_ID);
    Drone_data(k, :) = DronePos;
    
    
    
    % Capture non-filtered measurements
    nf_x(k) = DronePos(2);
    nf_y(k) = DronePos(3);
    nf_z(k) = DronePos(4);
    
    nf_v_x(k) = (nf_x(k) - prev_nf_x)/(1/120); % store non-filtered velocity
    prev_nf_x = nf_x(k); % Capture previous non-filtered position

    nf_v_y(k) = (nf_y(k) - prev_nf_y)/(1/120); % store non-filtered velocity
    prev_nf_y = nf_y(k); % Capture previous non-filtered position
    
    nf_v_z(k) = (nf_z(k) - prev_nf_z)/(1/120); % store non-filtered velocity
    prev_nf_z = nf_z(k); % Capture previous non-filtered position
    
    
    
    % Apply low pass filter to position measurements
    [x_f, lpfData_x] = lpf_2(lpfData_x, DronePos(2));
    p_x(k) = x_f;
    [vx_f, lpfData_vx] = lpf_2(lpfData_vx, (x_f - prev_x)/(1/120));
    f_v_x(k) = vx_f;
    prev_x = x_f;
    
    [y_f, lpfData_y] = lpf_2(lpfData_y, DronePos(3));
    p_y(k) = y_f;
    [vy_f, lpfData_vy] = lpf_2(lpfData_vy, (y_f - prev_y)/(1/120));
    f_v_y(k) = vy_f;
    prev_y = y_f;
    
    [z_f, lpfData_z] = lpf_2(lpfData_z, DronePos(4));
    p_z(k) = z_f;
    [vz_f, lpfData_vz] = lpf_2(lpfData_vz, (z_f - prev_z)/(1/120));
    f_v_z(k) = vz_f;
    prev_z = z_f;
end



%% QUIT
ik = 0;
while ik < 10
    write(joy_c, [0, 128, 0, 128, 128, 0, 0], 'uint8', "WithoutResponse");
    java.lang.Thread.sleep(100);
    ik = ik+1;
end
% Finally, Close the Motive Client
theClient.Uninitialize();

disp('Done')
%end

%% Plot Results
% Plot filtered position vs non-filtered position
figure()
plot(p_x, 'color', 'red')
hold on;
plot(nf_x, 'color', 'blue')
title("Position - X")
x_RMSE = sqrt(mean((nf_x - p_x).^2))  % Root Mean Squared Error

figure()
plot(p_y, 'color', 'red')
hold on;
plot(nf_y, 'color', 'blue')
title("Position - Y")
y_RMSE = sqrt(mean((nf_y - p_y).^2))  % Root Mean Squared Error

figure()
plot(p_z, 'color', 'red')
hold on;
plot(nf_z, 'color', 'blue')
title("Position - Z")
z_RMSE = sqrt(mean((nf_z - p_z).^2))  % Root Mean Squared Error

% Plot filtered velocity vs non-filtered velocity
figure()
plot(f_v_x, 'color', 'red')
hold on;
plot(nf_v_x, 'color', 'blue')
title("Velocity - X")
xv_RMSE = sqrt(mean((nf_v_x - f_v_x).^2))  % Root Mean Squared Error

figure()
plot(f_v_y, 'color', 'red')
hold on;
plot(nf_v_y, 'color', 'blue')
title("Velocity - Y")
yv_RMSE = sqrt(mean((nf_v_y - f_v_y).^2))  % Root Mean Squared Error

figure()
plot(f_v_z, 'color', 'red')
hold on;
plot(nf_v_z, 'color', 'blue')
title("Velocity - Z")
zv_RMSE = sqrt(mean((nf_v_z - f_v_z).^2))  % Root Mean Squared Error

% XCORR of Z velocity
% figure()
% [c,lags] = xcorr(nf_v_z,f_v_z);
% stem(lags,c)


% Plot 3d position
figure()
plot3(p_x, p_y, p_z)
grid on

