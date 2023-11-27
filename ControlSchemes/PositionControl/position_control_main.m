% function position_control_main = position_control_main(in1, in2) 
clear;
clc;
close all;

%% Documentation variables
ExperimentNum = 0;
DroneNum = 0;
ExperimentType = "P";

% load("AFOSR_Results\1-C_Metadata.mat",'ExperimentNum','DroneNum','ExperimentType');   
% ExperimentNum = ExperimentNum + 1;
% expStr = int2str(ExperimentNum);
% droneStr = int2str(DroneNum);
% filename = sprintf("AFOSR_Results/%s-%s_Metadata", droneStr, ExperimentType);
% save(filename, 'ExperimentNum', 'DroneNum', 'ExperimentType');
% timeNow = "mm-dd-yy_HH-MM";
% filenameDate = datestr(now, timeNow);
% filename = sprintf("AFOSR_Results/%s-%s-%s_%s",droneStr,ExperimentType,expStr, filenameDate);


% 2-C-3_mm-dd-yy_HH-MM
% DroneID-TypeofExp-ExpNum_Date_Time

if(ExperimentType == "C")
    TRAJECTORY = 0; % 0 = circular, 1 = origin reference
else
    TRAJECTORY = 1; % 1 = point, 1 = origin reference
end
TRAJECTORY = 1;

%% Setting up data transfer memory share
memshare_filename = fullfile(tempdir, 'position_control_memshare.dat');

% Open the memshare file
[f, msg] = fopen(memshare_filename, 'w');
if f ~= -1
    fwrite(f, zeros(1,256), 'double');
    fclose(f);
else
    error('MATLAB:demo:send:cannotOpenFile', ...
          'Cannot open file "%s": %s.', memshare_filename, msg);
end
%     end
 
% Memory map the file.
memMap = memmapfile(memshare_filename, 'Writable', true, 'Format', 'double');


%% Instantiate client object to run Motive API commands
dllPath = fullfile('d:','StDroneControl','NatNetSDK','lib','x64','NatNetML.dll');
mocapHandle = MocapAPI();
mocapHandle.init(dllPath)

%% Connect to the Drone via Radio & BLE
commsHandle = Communications();

% Create bluetooth connection
b = ble("C02835321733"); % ST DRONE FRAME 1
% b = ble("C0286e325133"); % ST DRONE FRAME 2
ble_imu_char = characteristic(b, "00000000-0001-11E1-9AB4-0002A5D5C51B" , "00E00000-0001-11E1-AC36-0002A5D5C51B"); % Read IMU
ble_arm_char = characteristic(b, "00000000-0001-11E1-9AB4-0002A5D5C51B" , "20000000-0001-11E1-AC36-0002A5D5C51B"); % Read arming status
ble_bat_char = characteristic(b, "00000000-0001-11E1-9AB4-0002A5D5C51B" , "001D0000-0001-11E1-AC36-0002A5D5C51B"); % Read battery/pressure status

subscribe(ble_imu_char)
ble_imu_char.DataAvailableFcn = @saveImuData;

% subscribe(ble_bat_char)
% ble_bat_char.DataAvailableFcn = @saveBatData;


% Open serial port for HC12 connection
device = serialport("COM3",38400);%19200
flush(device)


disp("Successfully established bluetooth and hc12 connections")

%% Load XBox Controller
xboxControllerHandle = XboxController();
xboxControllerHandle.init()
calibrate=0;arm=0;override=0;land=0;setpointMode=0;setpointPrev=0;setpointNext=0;

disp("Generated Xbox controller interface")

%% Load waypoints
waypointsHandle = WaypointGenerator();

%% Set up data collection vectors
WARMUP = 250; % Filter warmup time period

% Data collection vectors
Drone_data = [];%zeros(ITERATIONS + 2, 7);  % drone position data
sent_data = [];%zeros(ITERATIONS + 1, 3); % data sent to drone
cont_actual_data = [];%zeros(ITERATIONS + 1, 9);
errors = [];
desired_attitudes = [];%zeros(ITERATIONS + 1, 2);

% X,Y,Z position data
p_x = [];%zeros(ITERATIONS + 1, 1);
p_y = [];%zeros(ITERATIONS + 1, 1);
p_z = [];%zeros(ITERATIONS + 1, 1);

% Filtered velocity data
f_v_x = [];%zeros(ITERATIONS + 1, 1);
f_v_y = [];%zeros(ITERATIONS + 1, 1);
f_v_z = [];%zeros(ITERATIONS + 1, 1);

% Timing diagnostics
wTimes          = [];
rTimes          = [];
loopTimes       = [];
sleepTimes      = [];
getPosTimes     = [];
memShareTimes   = [];
xboxTimes       = [];

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
[DronePos] = mocapHandle.GetDronePosition();
Drone_data(1, :) = DronePos;

%% SET POINT TO TRACK
[x_ref, y_ref, z_ref] = waypointsHandle.getWaypoint();
comm_yaw_d = 128; % integer representation of 128 is 0 degrees. Min max is 30 degrees
psi_d = 0;

%% Initialize lowpass filter
[DronePos] = mocapHandle.GetDronePosition();
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
disp("Running warmup filter")
for i = 1:WARMUP
    [DronePos] = mocapHandle.GetDronePosition();
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

%% Battery level
% d = zeros(1,12); % For reading IMU
% timestamps = datetime(zeros(1,1), 0, 0); %a 10x1 array of datetime
% [d(1,:), timestamps(1)] = commsHandle.readBLE(ble_bat_char)
% fprintf('Battery level: %f%%\n', d(1,1)/3); % divide by ration so 3.2 = 0%


%% Wait for drone to be armed
disp("Calibrate/arm drone to start autonomous flight")
data = zeros(1,3); % For reading IMU
timestamps = datetime(zeros(1,1), 0, 0); %a 10x1 array of datetime


controlMode = 0;
% controlMode = 1;
% commsHandle.sendDataUpdatePacket(device,commsHandle.DR_UPDATE_CM, controlMode);


toggleArm = 0;
while(1)
    % Check if status onboard is armed
    [data(1,:), timestamps(1)] = commsHandle.readBLE(ble_arm_char);
    if(data(1,3) == 1)
        toggleArm = 1;
        break;
    end
    
    % Check xbox input and send arm command when 'RB' is pressed
    prevCalibrate=calibrate;prevArm=arm;prevOverride=override;prevLand=land;prevSetpointMode=setpointMode;prevSetpointPrev=setpointPrev;prevSetpointNext=setpointNext;
    [xbox_comm_thrust,xbox_comm_yaw,xbox_comm_pitch,xbox_comm_roll,calibrate,arm,override,land, ...
        setpointMode,setpointPrev,setpointNext] = xboxControllerHandle.getState();
    if(arm && ~prevArm)
        disp("Arming")
        commsHandle.sendDataUpdatePacket(device,commsHandle.DR_UPDATE_ARM, 1);
    end
    if(calibrate && ~prevCalibrate)
        disp("Calibrating")
        commsHandle.sendDataUpdatePacket(device,commsHandle.DR_UPDATE_CAL, 1);
    end

    pause(0.1);
end


%% Run Position Control Loop
disp("Starting in 1 second...")
% java.lang.Thread.sleep(1*1000);
pause(1);

T_trim = 130;
roll_trim = 0; % roll left is negative
pitch_trim = 0;

k = 1;
dT = 1/60; % 55Hz (writing only) - look into if dT might be faster 

[prevDronePos] = mocapHandle.GetDronePosition();
Drone_attitude_data = [];
Drone_rate_data = [];
packetCount = [];
ahrsRec = [];
ahrsRec2 = [];
ahrsRateRec = [];
eulerCmdRec = [];
thrusts = [];
accRec = [];
gyrRec = [];
xRefs = [];
yRefs = [];
zRefs = [];
Rhat = [];
FullState = [];
pidAtt_errors = [];
commsFromDrone = [];
error_code = [0];
error_code2 = [0];
error_code3 = [0];
% controlMode = 0;
landingFlag = 0;


global data;
global timestamp;
data = zeros(1,20); % For reading IMU
timestamp = datetime(zeros(1,1), 0, 0); %a 10x1 array of datetime

global dataBat;
global timestampBat;
dataBat = zeros(1,12); % For reading IMU
timestampBat = datetime(zeros(1,1), 0, 0); %a 10x1 array of datetime

rec = [];
Tcmds = [];

totalTime = 0;
startT = tic; 
while(1)
    % Get new drone position and store
    startTPos = tic;
    [DronePos] = mocapHandle.GetDronePosition();
    getPosTimes(k) = toc(startTPos);    
    Drone_data(k+1, :) = DronePos;

    Drone_attitude_data(k,:) = DronePos(5:7)*180/pi;
    Drone_rate_data(k,:) = ((DronePos(5:7) - prevDronePos(5:7))*180/pi)/dT;
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
    
    % Trajectory decision while flying
    if(TRAJECTORY == 0)
        % Circular trajectory
        x_ref = 0.5*cos(0.01*k);
        y_ref = 0.5*sin(0.01*k);
    else
        % Position hover trajectory
        [x_ref, y_ref, z_ref] = waypointsHandle.getWaypoint();
    end

    % Landing condition
    if(landingFlag)
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
    
    % Get latest xbox controller input
    xboxTime = tic;
    prevCalibrate=calibrate;prevArm=arm;prevOverride=override;prevLand=land;prevSetpointMode=setpointMode;prevSetpointPrev=setpointPrev;prevSetpointNext=setpointNext;
    [xbox_comm_thrust,xbox_comm_yaw,xbox_comm_pitch,xbox_comm_roll,calibrate,arm,override,land, ...
        setpointMode,setpointPrev,setpointNext] = xboxControllerHandle.getState();
    xboxTimes(k) = toc(xboxTime);

    % Send updates/commands to the Drone
    wTime = tic;
    
    % Send data update
    if(override && ~prevOverride)
        disp("Toggling override")
        controlMode = xor(controlMode,1); % Toggle AOMC (0) and MOMC (1)
        commsHandle.sendDataUpdatePacket(device,commsHandle.DR_UPDATE_CM, controlMode);
    end
    if(arm && ~prevArm)
        disp("Toggling arm command")
        toggleArm = xor(toggleArm,1);
        commsHandle.sendDataUpdatePacket(device,commsHandle.DR_UPDATE_ARM, toggleArm);
    end
    if(calibrate && ~prevCalibrate)
        disp("Calibrating")
        commsHandle.sendDataUpdatePacket(device,commsHandle.DR_UPDATE_CAL, 1);
    end
    if(land && ~prevLand)
        disp("Beginning landing sequence")
        landingFlag = 1;
    end
    if(setpointPrev && ~prevSetpointPrev)
        waypointsHandle.prevWaypoint()
        [x_ref, y_ref, z_ref] = waypointsHandle.getWaypoint();
        fprintf('Setpoint change: [%.2f,%.2f,%.2f]\n', x_ref,y_ref,z_ref);
    end
    if(setpointNext && ~prevSetpointNext)
        waypointsHandle.nextWaypoint()
        [x_ref, y_ref, z_ref] = waypointsHandle.getWaypoint();
        fprintf('Setpoint change: [%.2f,%.2f,%.2f]\n', x_ref,y_ref,z_ref);
    end


    % Send attitude command
    if(controlMode == 1)
        rollCmdTruth = (double(xbox_comm_roll)/255)*60 - 30;
        pitchCmdTruth = (double(xbox_comm_pitch)/255)*60 - 30;
        yawCmdTruth = (double(xbox_comm_yaw)/255)*60 - 30;
        commsHandle.sendAttitudeCmdPacket(device,xbox_comm_yaw,xbox_comm_thrust,xbox_comm_roll,xbox_comm_pitch);
    else
        rollCmdTruth = phi_d;
        pitchCmdTruth = theta_d;
        yawCmdTruth = 0;
        commsHandle.sendAttitudeCmdPacket(device,comm_yaw_d,comm_thr_d,comm_phi_d,comm_theta_d);
    end
    wTimes(k) = toc(wTime);


    % Thrust commands
    Tcmds(k) = typecast(uint8(data(1, 1:2)), 'uint16')/10;
    
    % Store on-board attitude estimate
    ahrsX_rate = commsHandle.parseBLE(data(1, 3:4),1000);%1
    ahrsY_rate = commsHandle.parseBLE(data(1, 5:6),1000);
    ahrsZ_rate = commsHandle.parseBLE(data(1, 7:8),1000);
    ahrsRateRec(k,:) = [ahrsX_rate, ahrsY_rate, ahrsZ_rate]; % AHRS


    % Store on-board attitude estimate
    ahrsX = ((data(1, 9) / 255) * 60) - 30;
    ahrsY = ((data(1, 11) / 255) * 60) - 30;
    ahrsZ = ((data(1, 13) / 255) * 60) - 30;
    ahrsRec(k,:) = [ahrsX, ahrsY, ahrsZ]; % AHRS


    pidX_error = ((data(1, 10) / 255) * 100) - 50;
    pidY_error = ((data(1, 12) / 255) * 100) - 50;
    pidZ_error = ((data(1, 14) / 255) * 100) - 50;
    pidAtt_errors(k,:) = [pidX_error, pidY_error, pidZ_error]; % AHRS

   
    pidX_deriv = ((data(1, 15) / 255) * 100) - 50;
    pidY_deriv = ((data(1, 17) / 255) * 100) - 50;
    pidZ_deriv = ((data(1, 19) / 255) * 100) - 50;
    pidAtt_derivatives(k,:) =  [pidX_deriv, pidY_deriv, pidZ_deriv];

     
    pidX_integ = ((data(1, 16) / 255) * 40) - 20;
    pidY_integ = ((data(1, 18) / 255) * 40) - 20;
    pidZ_integ = ((data(1, 20) / 255) * 40) - 20;
    pidAtt_integrals(k,:) =  [pidX_integ, pidY_integ, pidZ_integ];




%      ahrsX_Rate = commsHandle.parseBLE(dataBat(1, 7:8),10);%1
%      ahrsY_Rate = commsHandle.parseBLE(dataBat(1, 9:10),10);
%      ahrsZ_Rate = commsHandle.parseBLE(dataBat(1, 11:12),10);
%      ahrsRateRec(k,:) = [ahrsX_Rate, ahrsY_Rate, ahrsZ_Rate]; % AHRS

    pwm1 = commsHandle.parseBLE(data(1,9:10),1);
    pwm2 = commsHandle.parseBLE(data(1,11:12),1);
    pwm3 = commsHandle.parseBLE(data(1,13:14),1);
    pwm4 = commsHandle.parseBLE(data(1,15:16),1);
    pwmSignals(k,:) = [pwm1, pwm2, pwm3, pwm4];

%     % Store on-board attitude commands
%      attCmdX = commsHandle.parseBLE(data(1,17:18),10);%100
%      attCmdY = commsHandle.parseBLE(data(1,19:20),10);
%      attCmdZ = commsHandle.parseBLE(data(1,13:14),10);
%      eulerCmdRec(k,:) = [attCmdX,attCmdY,attCmdZ]; % Commanded


     % Store packet count
     packetCount(k) = commsHandle.parseBLE(data(1,1:2),1);

     % Errir flag
%      error_code(k) = commsHandle.parseBLE(data(1,19:20),1);  % Sepflag - Transfer error
%      error_code2(k) = commsHandle.parseBLE(data(1,13:14),1); % Sepflag2 - FIFO error
%      error_code3(k) = commsHandle.parseBLE(data(1,11:12),1); % Sepflag3 - Direct Mode error
% 
%      t1 = commsHandle.parseBLE(data(1,15:16),1);
%      t2 = commsHandle.parseBLE(data(1,17:18),1); 
%      t3 = commsHandle.parseBLE(data(1,19:20),1);
%      torques(k,:) =  [t1, t2, t3];

     
    % Tylers data % --- (now - initTime)*100000

%     ahrsX_Rate,ahrsY_Rate,ahrsZ_Rate,
    totalTime = totalTime + dT; 
%     FullState(k,:) = [totalTime, x_ref,y_ref,z_ref,x_f,y_f,z_f,vx_f,vy_f,vz_f,rollCmdTruth,pitchCmdTruth,yawCmdTruth,ahrsX,ahrsY,ahrsZ,Drone_attitude_data(k,1),Drone_attitude_data(k,2),Drone_attitude_data(k,3),Drone_rate_data(k,1),Drone_rate_data(k,2),Drone_rate_data(k,3),pwmSignals(k,1),pwmSignals(k,2),pwmSignals(k,3),pwmSignals(k,4)];
    FullState(k,:) = [totalTime, x_ref,y_ref,z_ref,x_f,y_f,z_f,vx_f,vy_f,vz_f,pitchCmdTruth,rollCmdTruth,yawCmdTruth,Tcmds(k),ahrsX,ahrsY,ahrsZ,ahrsX_rate,ahrsY_rate,ahrsZ_rate,pidX_error,pidY_error,pidZ_error,pidX_deriv,pidY_deriv,pidZ_deriv,pidX_integ,pidY_integ,pidZ_integ,X_pid.x_curr_error,Y_pid.y_curr_error,Z_pid.z_curr_error,X_pid.deriv,Y_pid.deriv,Z_pid.deriv,X_pid.x_cumm_error,Y_pid.y_cumm_error,Z_pid.z_cumm_error,Drone_attitude_data(k,1),Drone_attitude_data(k,2),Drone_attitude_data(k,3),Drone_rate_data(k,1),Drone_rate_data(k,2),Drone_rate_data(k,3)];
    
    
    % Collect the data being sent
    errors(k) = Y_pid.y_curr_error;
    sent_data(k, :) = [comm_thr_d, comm_phi_d, comm_theta_d];
    cont_actual_data(k, :) = [pid_output_x, pid_output_y, pid_output_z];
    desired_attitudes(k, :) = [theta_d, phi_d];


    % Sleep delay 
    s = tic;
% %     java.lang.Thread.sleep(5); % 5ms delay
    pause(0.02); %0.02 (40HZ)  0.005 (100Hz)

    sleepTimes(k) = toc(s);


    k = k+1;
end

% Shut off drone
disp("Shutting Down")
ik = 0;
while ik < 10
    %QUIT
    commsHandle.sendAttitudeCmdPacket(device,128,0,128,128);
%     java.lang.Thread.sleep(10);
    pause(0.01)
    ik = ik+1;
end

% Finally, Close the Motive Client
mocapHandle.shutdown();

% Saving Workspace to file
save(filename)

% Signal to UI that program is done
memMap.Data(3) = 1;

% Unsub from BLE
unsubscribe(ble_imu_char)
clear ble_imu_char;

disp('Done')





function saveImuData(src,evt)
    global data;
    global timestamp;
    [data,timestamp] = read(src,'oldest');
end


function saveBatData(src,evt)
    global dataBat;
    global timestampBat;
    [dataBat,timestampBat] = read(src,'oldest');
end










