%% Clear past data
close all;  
clear all;
clc;

%% Instantiate client object to run Motive API commands
% https://optitrack.com/software/natnet-sdk/

% Create Motive client object
dllPath = fullfile('d:','DroneControl','NatNetSDK','lib','x64','NatNetML.dll');
assemblyInfo = NET.addAssembly(dllPath); % Add API function calls
theClient = NatNetML.NatNetClientML(0);

% Create connection to localhost, data is now being streamed through client object
HostIP = '127.0.0.1';
theClient.Initialize(HostIP, HostIP); 

%% Connect to the Drone via Bluetooth
b = ble("C0285B324333"); % Make sure to get the the Drone's MAC address before running this code
char = b.Characteristics; % Get the characteristics of the Drone

%% Assign the rigid body id. Double check with motive that the rigid body ID is correct.
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
Drone_data = zeros(ITERATIONS + 2, 7);  % drone position data
sent_data = zeros(ITERATIONS + 1, 3); % data sent to drone
cont_actual_data = zeros(ITERATIONS + 1, 3);

% X,Y,Z position data
p_x = zeros(ITERATIONS + 1, 1);
p_y = zeros(ITERATIONS + 1, 1);
p_z = zeros(ITERATIONS + 1, 1);

% X,Y,Z position data (non-filtered)
nf_x = zeros(ITERATIONS + 1, 1);
nf_y= zeros(ITERATIONS + 1, 1);
nf_z = zeros(ITERATIONS + 1, 1);

% Filtered velocity data
f_v_x = zeros(ITERATIONS + 1, 1);
f_v_y = zeros(ITERATIONS + 1, 1);
f_v_z = zeros(ITERATIONS + 1, 1);

% X,Y,Z velocity data (non-filtered)
nf_v_x = zeros(ITERATIONS + 1, 1);
nf_v_y = zeros(ITERATIONS + 1, 1);
nf_v_z = zeros(ITERATIONS + 1, 1);

% Velocity/position reference in upward direction
vz_ref = zeros(ITERATIONS + 1, 1);
zz_ref = zeros(ITERATIONS + 1, 1);


% PID vel error derivative filtered
global f_v_ex;
global f_v_ey;
global f_v_ez;
f_v_ex = zeros(ITERATIONS + 1, 1);
f_v_ey = zeros(ITERATIONS + 1, 1);
f_v_ez = zeros(ITERATIONS + 1, 1);

% PID vel error derivative non-filtered
global nf_v_ex;
global nf_v_ey;
global nf_v_ez;
nf_v_ex = zeros(ITERATIONS + 1, 1);
nf_v_ey = zeros(ITERATIONS + 1, 1);
nf_v_ez = zeros(ITERATIONS + 1, 1);

global iter;
iter = 1;

% Angles ?
PHI = zeros(ITERATIONS + 1, 1);
THETA = zeros(ITERATIONS + 1, 1);

%% Frequencies
% What are these used for ?
OUT_FREQ = 30;
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

%% Keep track of previous position for computing velocity.
k = 1;
p_z_ref = z_ref; % p_z_ref = previous z_ref
motiveClientDelay = [];
java.lang.Thread.sleep(10*1000); % Java sleep is much more accurate than matlab's pause (sleep in ms)
% loopTimes = datetime(zeros(k+1,1), 0, 0); %A (k+1) x 1 array of datetime
% loopTimes2 = datetime(zeros(k+1,1), 0, 0); %A (k+1) x 1 array of datetime

loopTime = []; % dT
getPosTime = [];
bleWriteTime = [];
javaSleepTime = [];
while(k <= ITERATIONS)
    loopTimeStart = tic;
    if(k <= 800)
        z_ref = 1.0;%z_ref + 1/OUT_FREQ *0.25;
    else
        z_ref = z_ref - 1/OUT_FREQ*0.25;
    end
    
    zz_ref(k) = z_ref;
    v_z_ref = (z_ref - p_z_ref)/(1/OUT_FREQ); % Isnt this going to be zero until k>800? (1 - 1)/(1/out_freq)
    vz_ref(k) = v_z_ref;
    p_z_ref = z_ref;
    
    % Saturations
    if z_ref >= 1.0
        z_ref = 1.0;
    end
    
    if vz_ref(k) > 1.0
        vz_ref(k) = 1.0;
    elseif vz_ref(k) < -1.0
        vz_ref(k) = -1.0;   
    end
    
    % Get new drone position
    getPosTimeStart = tic;
    [DronePos] = GetDronePosition(theClient, Drone_ID); %-------------------------------------------1deg (worst case)
    getPosTime(k) = toc(getPosTimeStart);
    Drone_data(k+1, :) = DronePos;
%     loopTimes(k) = datetime('now');
%     motiveClientDelay(k) = theClient.SecondsSinceHostTimestamp(DronePos(1));
    
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
    
    
    % ------------------------------------------------ about 40ms from filter but the position does not change much in 40ms
    
    % Call the XY controllers - spits out desired x,y acceleration (which is converted into desired angles below)
    [ddot_x_d, out_x, X_pid, ddot_y_d, out_y, Y_pid] = position_controller(X_pid, x_ref, x_f, 0.0, Y_pid, y_ref, y_f, 0.0, 1/OUT_FREQ);
  
    % Call the Z Controller and retrive the thrust value
    [T, out_z, Z_pid] = VZcont(Z_pid, z_ref, z_f, 0.0, 1/OUT_FREQ, k);
    
    % Convert desired accelerations to desired angles
    z_T = 0.0043*double(T) + 0.001;
    if z_f < 0.1 && k < 150
        phi_d = 0;
        theta_d = 0;
        X_pid.x_cumm_error = 0.0;
        Y_pid.y_cumm_error = 0.0;
    else
        % What do these angles correspond to?
        phi_d = -m/z_T * (ddot_x_d*cos(psi) + ddot_y_d*sin(psi))* 180/pi;
        theta_d = m/z_T * (-ddot_y_d*cos(psi) + ddot_x_d*sin(psi)) * 180/pi;
    end
    
    % Cap the angles
    phi_d = min(max(-MAX_ANGLE, phi_d), MAX_ANGLE);
    theta_d = min(max(-MAX_ANGLE, theta_d), MAX_ANGLE);
    
    PHI(k) = phi_d;
    THETA(k) = theta_d;
    
    % Convert the angles to 0 - 255
    slope_m = 255.0/(MAX_ANGLE - -MAX_ANGLE);
    comm_phi_d = uint8(slope_m *(phi_d + MAX_ANGLE));
    comm_theta_d = uint8(slope_m *(theta_d + MAX_ANGLE));
    
    % Collect the data being sent
    sent_data(k, :) = [T, comm_phi_d, comm_theta_d];
    cont_actual_data(k, :) = [out_x, out_y, out_z];
   
    
    %Send the command to the Drone
    bleWriteTimeStart = tic;
    write(joy_c, [0, 128, T, comm_phi_d, comm_theta_d, 0, 5], 'uint8', "WithoutResponse"); % ~18ms
    bleWriteTime(k) = toc(bleWriteTimeStart);
    
%     javaSleepTimeStart = tic;
%     java.lang.Thread.sleep(1/OUT_FREQ*1000); % 16.6ms
%     javaSleepTime(k) = toc(javaSleepTimeStart);
    
    k = k+1;
    iter = iter+1;
    loopTime(k) = toc(loopTimeStart);
end

ik = 0;
while ik < 10
    %% QUIT
    write(joy_c, [0, 128, 0, 128, 128, 0, 0], 'uint8', "WithoutResponse");
    java.lang.Thread.sleep(1/OUT_FREQ*1000);
    ik = ik+1;
end
% Finally, Close the Motive Client
theClient.Uninitialize();

disp('Done')
%end


% figure()
% plot(p_x, 'color', 'red')
% hold on
% plot(PHI, 'color', 'green')
% grid on
% 
% figure()
% plot(p_y, 'color', 'red')
% hold on
% plot(THETA, 'color', 'green')
% grid on
% 
% figure()
% plot(p_x(1), p_y(1), '*')
% hold on
% plot(p_x, p_y)
% axis([-1 1 -1 1])
% grid on

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


%%%%%%%%%%%%%% ERROR DERIVATIVE FILTERS %%%%%%%%%%%%%%%%


% Plot filtered position vs non-filtered position
figure()
plot(f_v_ex, 'color', 'red')
hold on;
plot(nf_v_ex, 'color', 'blue')
title("Vel Error Derivative - X")
x_error_RMSE = sqrt(mean((nf_v_ex - f_v_ex).^2))  % Root Mean Squared Error

figure()
plot(f_v_ey, 'color', 'red')
hold on;
plot(nf_v_ey, 'color', 'blue')
title("Vel Error Derivative - Y")
y_error_RMSE = sqrt(mean((nf_v_ey - f_v_ey).^2))  % Root Mean Squared Error

figure()
plot(f_v_ey, 'color', 'red')
hold on;
plot(nf_v_ey, 'color', 'blue')
title("Vel Error Derivative - Z")
z_error_RMSE = sqrt(mean((nf_v_ey - f_v_ey).^2))  % Root Mean Squared Error






% 
% epochTimes = convertTo(loopTimes,'epochtime','Epoch','2001-01-01','TicksPerSecond',1000);
% epochTimes2 = convertTo(loopTimes2,'epochtime','Epoch','2001-01-01','TicksPerSecond',1000);
% mean(epochTimes2-epochTimes)
mean(loopTime)
mean(getPosTime)
mean(bleWriteTime)
mean(javaSleepTime)

%%
tList = [];
for i=1:1000
    frameData = theClient.GetLastFrameOfData();
    tList(i) = frameData.fTimestamp;
    java.lang.Thread.sleep(10);
end

