% function position_control_main = position_control_main(in1, in2)
clear;
clc;
close all;

% Lab Environment Setup
motion_capture = OptitrackNatNet();
disp("Motion capture interface initialized")

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
psi_d = 0;

%% Wait for drone to be armed
disp("Calibrate/arm drone to start autonomous flight")
timestamps = datetime(zeros(1, 1), 0, 0); %a 10x1 array of datetime

controlMode = 0;
landingFlag = 0;
toggleArm = 0;

return

%% Run Position Control Loop
disp("Starting in 1 second...")
pause(1);

T_trim = 130;
roll_trim = 0; % roll left is negative
pitch_trim = 0;

[prev_vehicle_state] = motion_capture.get_position([0 0 0.7;
                     0.7 0.7 0.7;
                     -0.7 0.7 0.7;
                     -0.7 -0.7 0.7;
                     0.7 -0.7 0.7;])
;

% configure control loop rate
rate_controller = rateControl(control_frequency_hz);

for loop_index = 1:inf
    if loop_index ~= 1
        dT = r.LastPeriod;
    end
    if lazyflie.armed == 0
        disp("Drone disarmed, exiting control loop")
        continue;
    end

    % Get new drone position and store
    vehicle_state = motion_capture.get_position(1, true);


    % TODO: should always get reference from waypoint generator, all logic should be subsumed there
    % Trajectory decision while flying
    % Position hover trajectory
    [x_ref, y_ref, z_ref] = waypointsHandle.getWaypoint();

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

    [ddot_x_d, ddot_y_d, gTHR] = PositionController.control([x_ref, y_ref, z_ref], [], dT)
    ref_thrust = gTHR + T_trim;

    % TODO: yaw control
    % Calculate desired roll,pitch angles - From Harsh Report
    psi = vehicle_state(7); % yaw
    phi_d = -m / single(ref_thrust) * (ddot_x_d * cos(psi) + ddot_y_d * sin(psi)) * 180 / pi; % MIGHT NEED TO REPLACE comm_thr_d with actual thrust sent to actuators on drone
    theta_d = m / single(ref_thrust) * (-ddot_y_d * cos(psi) + ddot_x_d * sin(psi)) * 180 / pi;

    % saturate roll and pitch angles
    phi_d = min(max(-MAX_ANGLE, phi_d), MAX_ANGLE);
    theta_d = min(max(-MAX_ANGLE, theta_d), MAX_ANGLE);

    % Convert the angles to 0 - 255
    slope_m = 255.0 / (MAX_ANGLE - -MAX_ANGLE);
    comm_phi_d = uint8(slope_m * (phi_d + MAX_ANGLE));
    comm_theta_d = uint8(slope_m * (theta_d + MAX_ANGLE));



    %{
        TODO: implement UI handler. Process commands like start, stop 
    %}

    lazyflie.control(1, psi_d, ref_thrust, comm_phi_d, comm_theta_d);
    
    prev_vehicle_state = vehicle_state;
    % Sleep until next loop
    waitfor(r);
end

% Shut off drone
% TODO: shutdown vehicle
% vehicle.shutdown()

% Signal to UI that program is done
disp('Done');