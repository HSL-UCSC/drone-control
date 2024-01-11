% function position_control_main = position_control_main(in1, in2)
clear;
clc;
close all;

% Lab Environment Setup
motion_capture = OptitrackNatNet();
disp("Motion capture interface initialized")

% Load XBox Controller
xboxControllerHandle = XboxController();
calibrate = 0; arm = 0; override = 0; land = 0; setpointMode = 0; setpointPrev = 0; setpointNext = 0;
disp("Generated Xbox controller interface")

% Load waypoints
waypointsHandle = WaypointGenerator();

% Frequencies
control_frequency_hz = 55;
dT = 1 / control_frequency_hz;
OUT_FREQ = control_frequency_hz;
CUT_OFF_FREQ_VEL = 10;
CUT_OFF_FREQ_POS = 10;

% Mass of the drone
m = 69.89/1000;
MAX_ANGLE = 30.0; % 27.73;

hc12_writer = HC12Writer("COM3", 38400);
ble_reader = BLEReader("C02835321733");
disp("Successfully established bluetooth and hc12 connections")

position_controller = PositionController(OUT_FREQ, CUT_OFF_FREQ_VEL);

lazyflie = LazyFlie(ble_reader, hc12_writer, position_controller);

%% SET POINT TO TRACK
[x_ref, y_ref, z_ref] = waypointsHandle.getWaypoint();
comm_yaw_d = 128; % integer representation of 128 is 0 degrees. Min max is 30 degrees
psi_d = 0;

%% Wait for drone to be armed
disp("Calibrate/arm drone to start autonomous flight")
timestamps = datetime(zeros(1, 1), 0, 0); %a 10x1 array of datetime

controlMode = 0;
landingFlag = 0;
toggleArm = 0;

return

% TODO: move this into a vehicle arming loop
ble_arming_data = zeros(1, 3); % For reading IMU
for loop_index = 1:inf
    % Check if status onboard is armed
    [ble_arming_data(1, :), timestamps(1)] = lazyFlie.state_reader.read(ble_arm_char);

    if (ble_arming_data(1, 3) == 1)
        toggleArm = 1;
        break;
    end

    % Check xbox input and send arm command when 'RB' is pressed
    prevCalibrate = calibrate; prevArm = arm; prevOverride = override; prevLand = land; prevSetpointMode = setpointMode; prevSetpointPrev = setpointPrev; prevSetpointNext = setpointNext;
    [xbox_comm_thrust, xbox_comm_yaw, xbox_comm_pitch, xbox_comm_roll, calibrate, arm, override, land, ...
         setpointMode, setpointPrev, setpointNext] = xboxControllerHandle.getState();

    if (arm && ~prevArm)
        disp("Arming")
        lazyflie.sendDataUpdatePacket(device, lazyflie.ARM, 1);
    end

    if (calibrate && ~prevCalibrate)
        disp("Calibrating")
        lazyflie.sendDataUpdatePacket(device, lazyflie.CALIBRATION, 1);
    end

    pause(0.1);
end

%% Run Position Control Loop
disp("Starting in 1 second...")
pause(1);

T_trim = 130;
roll_trim = 0; % roll left is negative
pitch_trim = 0;

[prev_vehicle_state] = motion_capture.get_position();

% configure control loop rate
rate_controller = rateControl(control_frequency_hz);

for loop_index = 1:inf
    if loop_index ~= 1
        dT = r.LastPeriod;
    end
    
    % Get new drone position and store
    vehicle_state = motion_capture.get_position(1, true);


    % TODO: should always get reference from waypoint generator, all logic should be subsumed there
    % Trajectory decision while flying
    if (TRAJECTORY == 0)
        % Circular trajectory
        x_ref = 0.5 * cos(0.01 * loop_index);
        y_ref = 0.5 * sin(0.01 * loop_index);
    else
        % Position hover trajectory
        [x_ref, y_ref, z_ref] = waypointsHandle.getWaypoint();
    end

    % Landing condition
    if (landingFlag)
        x_ref = 0;
        y_ref = 0;
        z_ref = z_f - 0.10;

        if (z_f < 0.1)
            disp("Landed")
            break;
        end

    end

    [ddot_x_d, ddot_y_d, gTHR] = PositionController.control([x_ref, y_ref, z_ref], [x_f, y_ref, z_ref], dT)
    comm_thr_d = gTHR + T_trim;

    % TODO: yaw control
    % Calculate desired roll,pitch angles - From Harsh Report
    psi = vehicle_state(7); % yaw
    phi_d = -m / single(comm_thr_d) * (ddot_x_d * cos(psi) + ddot_y_d * sin(psi)) * 180 / pi; % MIGHT NEED TO REPLACE comm_thr_d with actual thrust sent to actuators on drone
    theta_d = m / single(comm_thr_d) * (-ddot_y_d * cos(psi) + ddot_x_d * sin(psi)) * 180 / pi;

    % saturate roll and pitch angles
    phi_d = min(max(-MAX_ANGLE, phi_d), MAX_ANGLE);
    theta_d = min(max(-MAX_ANGLE, theta_d), MAX_ANGLE);

    % Convert the angles to 0 - 255
    slope_m = 255.0 / (MAX_ANGLE - -MAX_ANGLE);
    comm_phi_d = uint8(slope_m * (phi_d + MAX_ANGLE));
    comm_theta_d = uint8(slope_m * (theta_d + MAX_ANGLE));



    %{
        TODO: move this into xbox controller class
        START RC BLOCK ------------------------------------------
    %}

    % Get latest xbox controller input
    xboxTime = tic;
    prevCalibrate = calibrate; prevArm = arm; prevOverride = override; prevLand = land; prevSetpointMode = setpointMode; prevSetpointPrev = setpointPrev; prevSetpointNext = setpointNext;
    [xbox_comm_thrust, xbox_comm_yaw, xbox_comm_pitch, xbox_comm_roll, calibrate, arm, override, land, ...
         setpointMode, setpointPrev, setpointNext] = xboxControllerHandle.getState();
    xboxTimes(loop_index) = toc(xboxTime);

    % Send updates/commands to the Drone
    wTime = tic;

    % Send data update
    if (override && ~prevOverride)
        disp("Toggling override")
        controlMode = xor(controlMode, 1); % Toggle AOMC (0) and MOMC (1)
        lazyFlie.control_writer.write(dataUpdatePacket(lazyflie.CONTROL_MODE, controlMode));
    end

    if (arm && ~prevArm)
        disp("Toggling arm command")
        toggleArm = xor(toggleArm, 1);
        lazyFlie.control_writer.write(dataUpdatePacket(lazyflie.ARM, toggleArm));
    end

    if (calibrate && ~prevCalibrate)
        disp("Calibrating")
        lazyFlie.control_writer.write(dataUpdatePacket(lazyflie.CALIBRATION, 1));
    end

    if (land && ~prevLand)
        disp("Beginning landing sequence")
        landingFlag = 1;
    end

    if (setpointPrev && ~prevSetpointPrev)
        waypointsHandle.prevWaypoint()
        [x_ref, y_ref, z_ref] = waypointsHandle.getWaypoint();
        fprintf('Setpoint change: [%.2f,%.2f,%.2f]\n', x_ref, y_ref, z_ref);
    end

    if (setpointNext && ~prevSetpointNext)
        waypointsHandle.nextWaypoint()
        [x_ref, y_ref, z_ref] = waypointsHandle.getWaypoint();
        fprintf('Setpoint change: [%.2f,%.2f,%.2f]\n', x_ref, y_ref, z_ref);
    end


   
    %{
        END RC BLOCK  ------------------------------------------
    %}
    
    lazyflie.control()
    
    prev_vehicle_state = vehicle_state;
    % Sleep until next loop
    waitfor(r);
end

% Shut off drone
% TODO: shutdown vehicle
% vehicle.shutdown()

% Signal to UI that program is done
disp('Done');