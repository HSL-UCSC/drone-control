%% Clear past data
close all;  
clear all;
clc;

%% Documentation variables
ExperimentNum = 0;
DroneNum = 0;
ExperimentType = "P";

load("AFOSR_Results\1-C_Metadata.mat",'ExperimentNum','DroneNum','ExperimentType');

ExperimentNum = ExperimentNum + 1;

expStr = int2str(ExperimentNum);
droneStr = int2str(DroneNum);

filename = sprintf("AFOSR_Results/%s-%s_Metadata", droneStr, ExperimentType);

save(filename, 'ExperimentNum', 'DroneNum', 'ExperimentType');

timeNow = "mm-dd-yy_HH-MM";

filenameDate = datestr(now, timeNow);

filename = sprintf("AFOSR_Results/%s-%s-%s_%s",droneStr,ExperimentType,expStr, filenameDate);


% 2-C-3_mm-dd-yy_HH-MM
% DroneID-TypeofExp-ExpNum_Date_Time

if(ExperimentType == "C")
    TRAJECTORY = 0; % 0 = circular, 1 = origin reference
else
    TRAJECTORY = 1; % 0 = circular, 1 = origin reference
end
TRAJECTORY = 1;

%% Instantiate client object to run Motive API commands
% https://optitrack.com/software/natnet-sdk/

% Create Motive client object
dllPath = fullfile('d:','StDroneControl','NatNetSDK','lib','x64','NatNetML.dll');
assemblyInfo = NET.addAssembly(dllPath); % Add API function calls
theClient = NatNetML.NatNetClientML(0);

% Create connection to localhost, data is now being streamed through client object
HostIP = '127.0.0.1';
theClient.Initialize(HostIP, HostIP); 

%% Connect to the Drone via Radio
%Make sure to get the the Drone's MAC address before running this code -
b = ble("C02835321733"); % ST DRONE FRAME 1
% b = ble("C0286E325133"); % FOAM CORE FRAME 1

char = b.Characteristics; % Get the characteristics of the Drone

device = serialport("COM5",19200)
flush(device)
startByte = 245; 
endByte = 2;

%% Assign the rigid body id. Double check with motive that the rigid body ID is correct.killswitch

Drone_ID = 1;
%Drone_ID = 2;

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
joy_c = characteristic(b, "00000000-0001-11E1-9AB4-0002A5D5C51B" , "00008000-0001-11E1-AC36-0002A5D5C51B"); % Write w/out response
% joy_c_imu = characteristic(b, "00000000-0001-11E1-9AB4-0002A5D5C51B" , "00E00000-0001-11E1-AC36-0002A5D5C51B") % Read IMU
joy_c_imu = characteristic(b, "00000000-0001-11E1-9AB4-0002A5D5C51B" , "00E00000-0001-11E1-AC36-0002A5D5C51B") % Read IMU

%% Next calibrate/arm the drone
% Reference slides 25,26 from - https://www.st.com/content/ccc/resource/sales_and_marketing/presentation/product_presentation/group0/bd/cc/11/15/14/d4/4a/85/STEVAL-DRONE01_GETTING_STARTED_GUIDE/files/steval-drone01_getting_started_guide.pdf/jcr:content/translations/en.steval-drone01_getting_started_guide.pdf

% 1) Place drone down flat and press reset button to calibrate it
% 2) Arm the drone: 
write(joy_c, [22, 128, 0, 128, 128, 0, 4], 'uint8', "WithoutResponse");
java.lang.Thread.sleep(2*1000); % Java sleep is much more accurate than matlab's pause (sleep in ms)
% write(joy_c, [22, 128, 0, 128, 128, 0, 0], 'uint8', "WithoutResponse");

%% Set up data collection vectors
ITERATIONS = 900 % Main loop time period
WARMUP = 250; % Filter warmup time period

% Data collection vectors
Drone_data = zeros(ITERATIONS + 2, 7);  % drone position data
sent_data = zeros(ITERATIONS + 1, 3); % data sent to drone
cont_actual_data = zeros(ITERATIONS + 1, 9);
errors = [];
desired_attitudes =  zeros(ITERATIONS + 1, 2);

% X,Y,Z position data
p_x = zeros(ITERATIONS + 1, 1);
p_y = zeros(ITERATIONS + 1, 1);
p_z = zeros(ITERATIONS + 1, 1);

% Filtered velocity data
f_v_x = zeros(ITERATIONS + 1, 1);
f_v_y = zeros(ITERATIONS + 1, 1);
f_v_z = zeros(ITERATIONS + 1, 1);

% Timing diagnostics
wTimes          = [];
rTimes          = [];
loopTimes       = [];
sleepTimes      = [];
getPosTimes     = [];

%% Frequencies
OUT_FREQ = 60; % 60Hz write only
CUT_OFF_FREQ_VEL = 10;
CUT_OFF_FREQ_POS = 10;

%% Mass of the drone
m = 69.89/1000;
MAX_ANGLE = 30.0;% 27.73;

%% Inititalize the PID controllers
X_pid = PID_Controller.Xpid_init(1, OUT_FREQ, CUT_OFF_FREQ_VEL);
Y_pid = PID_Controller.Ypid_init(1, OUT_FREQ, CUT_OFF_FREQ_VEL);
Z_pid = PID_Controller.Zpid_init(1, OUT_FREQ, CUT_OFF_FREQ_VEL);

%% Get the Drone data
[DronePos] = MocapAPI.GetDronePosition(theClient, Drone_ID);
Drone_data(1, :) = DronePos;

%% SET POINT TO TRACK
x_ref = 0.0;
y_ref = 0.0;
z_ref_final = 0.6; % 0.5 meter (500mm) % 0.005
comm_yaw_d = 128; % integer representation of 128 is 0 degrees. Min max is 30 degrees


%% Initialize lowpass filter
[DronePos] = MocapAPI.GetDronePosition(theClient, Drone_ID);
lpfData_x = Filter.lpf_2_init(OUT_FREQ, CUT_OFF_FREQ_POS, DronePos(2));
lpfData_y = Filter.lpf_2_init(OUT_FREQ, CUT_OFF_FREQ_POS, DronePos(3));
lpfData_z = Filter.lpf_2_init(OUT_FREQ, CUT_OFF_FREQ_POS, DronePos(4));

lpfData_vx = Filter.lpf_2_init(OUT_FREQ, CUT_OFF_FREQ_VEL, 0);
lpfData_vy = Filter.lpf_2_init(OUT_FREQ, CUT_OFF_FREQ_VEL, 0);
lpfData_vz = Filter.lpf_2_init(OUT_FREQ, CUT_OFF_FREQ_VEL, 0);

prev_x =  DronePos(2);
prev_y =  DronePos(3);
prev_z =  DronePos(4);

%% Warm up Filter
for i = 1:WARMUP
    [DronePos] = MocapAPI.GetDronePosition(theClient, Drone_ID);
    %Drone_data = [Drone_data; DronePos];
    
    [x_f, lpfData_x] = Filter.lpf_2(lpfData_x, DronePos(2));
    [vx_f, lpfData_vx] = Filter.lpf_2(lpfData_vx, x_f - prev_x);
    prev_x = x_f;
    
    [y_f, lpfData_y] = Filter.lpf_2(lpfData_y, DronePos(3));
    [vy_f, lpfData_vy] = Filter.lpf_2(lpfData_vy, y_f - prev_y);
    prev_y = y_f;
    
    [z_f, lpfData_z] = Filter.lpf_2(lpfData_z, DronePos(4));
    [vz_f, lpfData_vz] = Filter.lpf_2(lpfData_vz, z_f - prev_z);
    prev_z = z_f;
    
end

%% Run Radio Check
disp("Running radio check (5 seconds)")
% for i=1:1000
%     [data, timestamps] = read(joy_c_imu, 'latest');
%     write(device,[startByte, 128, 0, 128, 128, endByte],"uint8") % comm_thr_d
%     java.lang.Thread.sleep(5); % 10ms delay
%     packetCount = data(15:16)
% end

%% Run Main Loop
disp("Starting")
java.lang.Thread.sleep(5*1000); % Wait 5 seconds

T_trim = 130;

k = 1;
dT = 1/60; % 55Hz (writing only) - look into if dT might be faster 

[prevDronePos] = MocapAPI.GetDronePosition(theClient, Drone_ID);
data = zeros(1,20); % For reading IMU
timestamps = datetime(zeros(10,1), 0, 0); %a 10x1 array of datetime
Drone_pos_data = [];
Drone_rate_data = [];
packetCount = [];
torques = [];
thrusts = [];

z_ref_final = 0.7;
xRefs = [];
yRefs = [];
zRefs = [];


flag1 = 0;
flag2 = 0;
flag3 = 0;

startT = tic; 
while(1)
    % Get new drone position and store
    startTPos = tic;
    [DronePos] = MocapAPI.GetDronePosition(theClient, Drone_ID);
    getPosTimes(k) = toc(startTPos);    
    Drone_data(k+1, :) = DronePos;
    
    Drone_pos_data(k,:) = DronePos(5:7);
    Drone_rate_data(k,:) = (DronePos(5:7) - prevDronePos(5:7))/dT;
    prevDronePos = DronePos;
    

    % Apply low pass filter to position/velocity measurements
    [x_f, lpfData_x] = Filter.lpf_2(lpfData_x, DronePos(2));
    p_x(k) = x_f;
    [vx_f, lpfData_vx] = Filter.lpf_2(lpfData_vx, (x_f - prev_x)/dT);
    f_v_x(k) = vx_f;
    prev_x = x_f;
    
    [y_f, lpfData_y] = Filter.lpf_2(lpfData_y, DronePos(3));
    p_y(k) = y_f;
    [vy_f, lpfData_vy] = Filter.lpf_2(lpfData_vy, (y_f - prev_y)/dT);
    f_v_y(k) = vy_f;
    prev_y = y_f;
    
    [z_f, lpfData_z] = Filter.lpf_2(lpfData_z, DronePos(4));
    p_z(k) = z_f;
    [vz_f, lpfData_vz] = Filter.lpf_2(lpfData_vz, (z_f - prev_z)/dT);
    f_v_z(k) = vz_f;
    prev_z = z_f;
    
    % Get dT loop time
    loopTimes(k) = toc(startT);
    dT = loopTimes(k);
    startT = tic;
    
    % Reference decision while flying
    if(TRAJECTORY == 0)
        % Circular trajectory
        x_ref = 0.5*cos(0.01*k);
        y_ref = 0.5*sin(0.01*k);
        z_ref = z_ref_final;
    else
        % Position hover trajectory
        x_ref = x_f - sign(x_f)*0.25;
        if(x_f < 0.25 && x_f > -0.25)
            flag1 = 1;
            x_ref = 0;
        end
        if(flag1==1)
            x_ref = 0;
        end

        y_ref = y_f - sign(y_f)*0.25;
        if(y_f < 0.25 && y_f > -0.25)
            flag2 = 1;
            y_ref = 0;
        end
        if(flag2==1)
            y_ref = 0;
        end

        z_ref = z_ref_final;
    end

    % Landing condition
    if(k > ITERATIONS)
        x_ref = 0;
        y_ref = 0;
        z_ref = z_f - 0.10;

        if(z_f < 0.1)
            disp("Landed")
            break;
        end
    end


    % Store the refs
    xRefs(k) = x_ref;
    % Call the X Controller - Desired Roll
    [ddot_x_d, pid_output_x, X_pid] = PID_Controller.Xcontroller(X_pid, x_ref, x_f, vx_f, dT);

    yRefs(k) = y_ref;
    % Call the Y Controller - Desired Pitch
    [ddot_y_d, pid_output_y, Y_pid] = PID_Controller.Ycontroller(Y_pid, y_ref, y_f, vy_f, dT);

    zRefs(k) = z_ref;
    % Call the Z Controller - Desired Thrust
    [gTHR, pid_output_z, Z_pid] = PID_Controller.Zcontroller(Z_pid, z_ref, z_f, vz_f, dT);
    
    % Apply trim input thrust
    comm_thr_d = gTHR + T_trim;
    
    % Calculate desired roll,pitch angles - From Harsh Report
    psi = DronePos(7); % yaw
    phi_d = -m/single(comm_thr_d) * (ddot_x_d*cos(psi) + ddot_y_d*sin(psi))* 180/pi; % MIGHT NEED TO REPLACE comm_thr_d with actual thrust sent to actuators on drone
    theta_d = m/single(comm_thr_d) * (-ddot_y_d*cos(psi) + ddot_x_d*sin(psi)) * 180/pi;
    
    % Cap angles
    phi_d = min(max(-MAX_ANGLE, phi_d), MAX_ANGLE);
    theta_d = min(max(-MAX_ANGLE, theta_d), MAX_ANGLE);
    
    % Convert the angles to 0 - 255
    slope_m = 255.0/(MAX_ANGLE - -MAX_ANGLE);
    comm_phi_d = uint8(slope_m *(phi_d + MAX_ANGLE));
    comm_theta_d = uint8(slope_m *(theta_d + MAX_ANGLE));
    


    % Send the command to the Drone %
    wTime = tic;
%     write(joy_c, [0, comm_yaw_d, comm_thr_d, comm_phi_d, comm_theta_d, 0, 5], 'uint8', "WithoutResponse") % ~18ms
    [data(1,:), timestamps(1)] = read(joy_c_imu, 'latest');
    write(device,[startByte, comm_yaw_d, comm_thr_d, comm_phi_d, comm_theta_d, endByte],"uint8")
    wTimes(k) = toc(wTime);
    
    
    
    
    
     % Store on-board attitudes
%      thx = parse_ble(data(1,3:4),10);
%      thy = parse_ble(data(1,5:6),10);
%      thz = parse_ble(data(1,7:8),10);
%      euler(k,:) = [thx,thy,thz];

     % Store on-board torques
     torqueX = parse_ble(data(1, 3:4),10);%1
     torqueY = parse_ble(data(1, 5:6),10);
     torqueZ = parse_ble(data(1, 7:8),10);
     torques(k,:) = [torqueX, torqueY, torqueZ]; % AHRS

     % Store on-board attitude rates
     thx_rate = parse_ble(data(1,9:10),10);%100
     thy_rate = parse_ble(data(1,11:12),10);
     thz_rate = parse_ble(data(1,13:14),10);
     euler_rates(k,:) = [thx_rate,thy_rate,thz_rate]; % Commanded

     % Store on-board packet count
     packetCount(k,:) = data(1,15:16);

     % Store on-board thrust
     thrust = parse_ble(data(1, 19:20),1);
     thrusts(k,:) = thrust;


    % Tylers data
    Tyler(k,:) = [x_f,y_f,z_f,vx_f,vy_f,vz_f,Drone_pos_data(k,1),Drone_pos_data(k,2),Drone_pos_data(k,3),Drone_rate_data(k,1),Drone_rate_data(k,2),Drone_rate_data(k,3),torques(k,1),torques(k,2),torques(k,3),thrust];
    
    
    % Collect the data being sent
    errors(k) = Y_pid.y_curr_error;
    sent_data(k, :) = [comm_thr_d, comm_phi_d, comm_theta_d];
    cont_actual_data(k, :) = [pid_output_x, pid_output_y, pid_output_z];
    desired_attitudes(k, :) = [theta_d, phi_d];

    s = tic;
    java.lang.Thread.sleep(5); % 5ms delay

    sleepTimes(k) = toc(s);


    k = k+1;
end

% Shut off drone
disp("Shutting Down")
ik = 0;
while ik < 10
    %QUIT
    write(device,[startByte, 128, 0, 128, 128, endByte],"uint8")
    java.lang.Thread.sleep(10);
    ik = ik+1;
end

% Finally, Close the Motive Client
theClient.Uninitialize();

disp('Done')
save(filename)


%% Display Results
% Saving Workspace to file


close all

% % Attitude tracking

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
close all

figure()
plot(loopTimes)
title("Loop time")

figure()
plot(packetCount)
title("Packet count")

figure()
hold on;
plot(-desired_attitudes(:,1));
plot(euler_rates(:,1))
plot(Drone_pos_data(:,1)*180/pi)
plot(torques(:,1))
title("Roll tracking")
legend("Sent Desired", "Actual Commanded", "MOCAP","AHRS")

figure()
hold on;
plot(desired_attitudes(:,2));
plot(euler_rates(:,2))
plot(Drone_pos_data(:,2)*180/pi)
plot(torques(:,2))
title("Pitch tracking")

legend("Sent Desired", "Actual Commanded", "MOCAP","AHRS")

% figure()
% hold on;
% plot(euler_rates(:,3))
% plot(torques(:,3))
% title("Yaw tracking")
% legend("Sent Desired", "Actual Commanded", "AHRS")