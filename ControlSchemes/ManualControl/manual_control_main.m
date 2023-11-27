% function position_control_main = position_control_main(in1, in2) 
clear;
clc;
close all;


%% Instantiate client object to run Motive API commands
dllPath = fullfile('d:','StDroneControl','NatNetSDK','lib','x64','NatNetML.dll');
mocapHandle = MocapAPI();
mocapHandle.init(dllPath)

%% Connect to the Drone via Radio & BLE
commsHandle = Communications();

% Create bluetooth connection
b = ble("C0286e325133"); % ST DRONE FRAME 2
ble_imu_char = characteristic(b, "00000000-0001-11E1-9AB4-0002A5D5C51B" , "00E00000-0001-11E1-AC36-0002A5D5C51B"); % Read IMU
ble_arm_char = characteristic(b, "00000000-0001-11E1-9AB4-0002A5D5C51B" , "20000000-0001-11E1-AC36-0002A5D5C51B"); % Read arming status
% ble_bat_char = characteristic(b, "00000000-0001-11E1-9AB4-0002A5D5C51B" , "001D0000-0001-11E1-AC36-0002A5D5C51B"); % Read battery/pressure status

subscribe(ble_imu_char)
ble_imu_char.DataAvailableFcn = @saveImuData;

% Open serial port for HC12 connection
device = serialport("COM3",38400);%19200
flush(device)


disp("Successfully established bluetooth and hc12 connections")

%% Load XBox Controller
xboxControllerHandle = XboxController();
xboxControllerHandle.init()
calibrate=0;arm=0;override=0;land=0;setpointMode=0;setpointPrev=0;setpointNext=0;

disp("Generated Xbox controller interface")

%% Frequencies
OUT_FREQ = 60; % 60Hz write only
CUT_OFF_FREQ_VEL = 10;
CUT_OFF_FREQ_POS = 10;


%% Set up data collection vectors
WARMUP = 250; % Filter warmup time period

% Data collection vectors
Drone_data = [];%zeros(ITERATIONS + 2, 7);  % drone position data
sent_data = [];%zeros(ITERATIONS + 1, 3); % data sent to drone
cont_actual_data = [];%zeros(ITERATIONS + 1, 9);
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

%% Get the Drone data
[DronePos] = mocapHandle.GetDronePosition();
Drone_data(1, :) = DronePos;


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


%% Wait for drone to be armed
disp("Calibrate/arm drone to start autonomous flight")
data2 = zeros(1,3); % For reading IMU
timestamps = datetime(zeros(1,1), 0, 0); %a 10x1 array of datetime


global data;
global timestamp;
data = zeros(1,20); % For reading IMU
timestamp = datetime(zeros(1,1), 0, 0); %a 10x1 array of datetime
rec = [];
rx = [];
ijk = 1;

toggleArm = 0;
while(1)
    % Check if status onboard is armed
    [data2(1,:), timestamps(1)] = commsHandle.readBLE(ble_arm_char);
    if(data2(1,3) == 1)
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
    rx(ijk) = commsHandle.parseBLE(data(1,19:20),1);
    ijk = ijk + 1;
    pause(0.1);
end


%% Run Manual Control Loop
disp("Enabling Manual Control")
controlMode = 1; % Toggle AOMC (0) and MOMC (1)
commsHandle.sendDataUpdatePacket(device,commsHandle.DR_UPDATE_CM, controlMode);


k = 1;
dT = 1/60; % 55Hz (writing only) - look into if dT might be faster 

[prevDronePos] = mocapHandle.GetDronePosition();
Drone_attitude_data = [];
Drone_rate_data = [];
packetCount = [];
ahrsRec = [];
ahrsRec2 = [];
eulerCmdRec = [];
thrusts = [];
accRec = [];
gyrRec = [];
xRefs = [];
yRefs = [];
zRefs = [];
Rhat = [];
FullState = [];

commsFromDrone = [];
error_code = [0];


startT = tic; 
while(1)
    % Get new drone position and store
    startTPos = tic;
    [DronePos] = mocapHandle.GetDronePosition();
    getPosTimes(k) = toc(startTPos);    
    Drone_data(k+1, :) = DronePos;
    
    Drone_attitude_data(k,:) = DronePos(5:7);
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
    


    % Get latest xbox controller input
    xboxTime = tic;
    prevCalibrate=calibrate;prevArm=arm;prevOverride=override;prevLand=land;prevSetpointMode=setpointMode;prevSetpointPrev=setpointPrev;prevSetpointNext=setpointNext;
    [xbox_comm_thrust,xbox_comm_yaw,xbox_comm_pitch,xbox_comm_roll,calibrate,arm,override,land, ...
        setpointMode,setpointPrev,setpointNext] = xboxControllerHandle.getState();
    xboxTimes(k) = toc(xboxTime);

    % Check for flight done signal
    if(land && ~prevLand)
        disp("Flight Complete")
        break;
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

    % Send updates/commands to the Drone
    wTime = tic;
    commsHandle.sendAttitudeCmdPacket(device,xbox_comm_yaw,xbox_comm_thrust,xbox_comm_roll,xbox_comm_pitch);
    wTimes(k) = toc(wTime);
    
    
     % Store on-board packet count
     packetCount(k,:) = data(1,15:16);
    
     rx(k) = commsHandle.parseBLE(data(1,19:20),1);
    
    % Collect the data being sent
    sent_data(k, :) = [xbox_comm_thrust, xbox_comm_roll, xbox_comm_pitch, xbox_comm_yaw];


    % Sleep delay 
    s = tic;
    pause(0.01)
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

% Unsub from BLE
unsubscribe(ble_imu_char)
clear ble_imu_char;

disp('Done')





function saveImuData(src,evt)
    global data;
    global timestamp;
    [data,timestamp] = read(src,'oldest');
end







