% function position_control_main = position_control_main(in1, in2)
clear;
% clc;
% close all;

% Lab Environment Setup
% todo: implement swith to select one of Vicon or Optitrack
motion_capture = Vicon.Client();
motion_capture.initialize();

disp("Motion capture interface initialized")

% waypoints handle is a concrete implementation of the Path interface
waypointsHandle = Path.EventPath([0 0 0.7]);

% Frequencies
control_frequency_hz = 120;
dT = 1 / control_frequency_hz;
OUT_FREQ = control_frequency_hz;
CUT_OFF_FREQ_VEL = 10;
CUT_OFF_FREQ_POS = 10;

% Mass of the drone
% TODO!!!!!!: match mass of quad under test - set this in vehicle model implementations
vehicle = Betaflight("COM24", 115200);
m = 60;
MAX_ANGLE = 15.0;

position_controller = Control.PositionController(OUT_FREQ, CUT_OFF_FREQ_VEL);

% % todo: consider moving this to the LazyFlie constructor
% lazyflie = LazyFlie("COM3", 38400, "C02835321733", position_controller);
% disp("Successfully established bluetooth and hc12 connections")
% disp("Calibrate/arm drone to start autonomous flight")


%% SET POINT TO TRACK
[x_ref, y_ref, z_ref] = waypointsHandle.get_waypoint();
comm_yaw_d = 0;
psi_d = 0;

timestamps = datetime(zeros(1, 1), 0, 0); %a 10x1 array of datetime

controlMode = 0;
landingFlag = 0;
toggleArm = 0;

%% Run Position Control Loop
disp("Starting in 1 second...")
pause(1);
vehicle.arm()

T_trim = .5;
roll_trim = 0;
pitch_trim = 0;

vehicle_state = motion_capture.get_pose("mob6", "mob6");
p = vehicle_state.translation;
R = vehicle_state.rotation;
[xprev, yprev, zprev, phiprev, thetaprev, psiprev] = deal(p{1}, -p{2}, p{3}, R{1}, R{2}, R{3});

% configure control loop rate
rate_controller = rateControl(control_frequency_hz);
gTHR = 0;

for loop_index = 1:inf

    if loop_index ~= 1
        dT = rate_controller.LastPeriod;
    end

    if vehicle.armed == 0
        res = input('');
        disp(res);
        vehicle.armed = 1;
    end

    % Get new drone position and store
    vehicle_state = motion_capture.get_pose("mob6", "mob6");
    p = vehicle_state.translation;
    R = vehicle_state.rotation;
    [x, y, z, phi, theta, psi] = deal(p{1}, -p{2}, p{3}, R{1}, R{2}, R{3});
    % TODO: should always get reference from waypoint generator, all logic should be subsumed there
    % TODO: implement landing logic in waypoint generator
    [x_ref, y_ref, z_ref] = waypointsHandle.get_waypoint([x, y, z, phi, theta, psi], landingFlag);

    comm_thr_d = gTHR + T_trim;
    [ddot_x_d, ddot_y_d, gTHR] = position_controller.control([x_ref, y_ref, z_ref], [x/1000, y/1000, z/1000], dT);

    % TODO: yaw control
    % Calculate desired roll,pitch angles - From Harsh Report
    % psi = vehicle_state(7); % yaw
    phi_d = -m / single(comm_thr_d) * (ddot_x_d * cos(psi) + ddot_y_d * sin(psi)) * 180 / pi;
    theta_d = m / single(comm_thr_d) * (-ddot_y_d * cos(psi) + ddot_x_d * sin(psi)) * 180 / pi;

    % saturate roll and pitch angles
    phi_d = min(max(-MAX_ANGLE, phi_d), MAX_ANGLE);
    theta_d = min(max(-MAX_ANGLE, theta_d), MAX_ANGLE);

    % Convert the angles to 0 - 255
    % slope_m = 255.0 / (MAX_ANGLE - -MAX_ANGLE);
    comm_phi_d = phi_d;
    comm_theta_d = theta_d;
    %{
        TODO: implement UI handler. Process commands like start, stop
    %}
    vehicle.control(z_ref, comm_phi_d, comm_theta_d, psi_d);
    xprev = x;
    yprev = y;
    zprev = z;
    phiprev = phi;
    thetaprev = theta;
    psiprev = psi;
    % Sleep until next loop
    waitfor(rate_controller);
end

% Shut off drone
% TODO: shutdown vehicle
% vehicle.shutdown()

% Signal to UI that program is done
disp('Done');
