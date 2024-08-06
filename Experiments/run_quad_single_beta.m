clear motion_capture;
clear vehicles vehicle;
ports = serialportlist

% Lab Environment Setup
vehicle = Betaflight("COM24", 115200, "mob6");
motion_capture = Vicon.Client();

%% make the vehicle id the same name as motion capture EZ
% vehicle = Quadsim('127.0.0.1', 25000, 'sim1');
% vehicles = [vehicle];
% motion_capture = Localization.QuadsimMotionCapture(vehicles);
waypointsHandle = Path.EventPath([0 0 .7]);
control_frequency_hz = 120;
dT = 1 / control_frequency_hz;


motion_capture.initialize();
disp("Motion capture interface initialized")

% waypoints handle is a concrete implementation of the Path interface
MAX_ANGLE = 12.0;

position_controller = Control.PositionController(control_frequency_hz, 10);

%% SET POINT TO TRACK
[x_ref, y_ref, z_ref] = waypointsHandle.get_waypoint();
psi_d = 0;

timestamps = datetime(zeros(1, 1), 0, 0); %a 10x1 array of datetime

controlMode = 0;
landingFlag = 0;
toggleArm = 0;

%% Run Position Control Loop
disp("Arm the vehicle. Starting control loop in 5 seconds")
vehicle.arm()
pause(1);

T_trim = .4;
roll_trim = 0;
pitch_trim = 0;

vehicle_state = motion_capture.get_pose(vehicle.id);
p = vehicle_state.translation;
R = vehicle_state.rotation;
[xprev, yprev, zprev, phiprev, thetaprev, psiprev] = deal(p(1), p(2), p(3), R(1), R(2), R(3));

% configure control loop rate
rate_controller = rateControl(control_frequency_hz);
waitfor(rate_controller);
gTHR = .4;

while 1
  
  dT = rate_controller.LastPeriod;
  
  if vehicle.armed == 0
    res = input('');
    disp(res);
    vehicle.armed = 1;
  end
  
  % Get new drone position and store
  vehicle_state = motion_capture.get_pose(vehicle.id);
  p = vehicle_state.translation / 1000;
  R = vehicle_state.rotation;
  v = vehicle_state.velocity;
  [x, y, z, phi, theta, psi, u, v, w] = deal(p(1), p(2), p(3), R(1), R(2), R(3), v(1), v(2), v(3));
  % TODO: implement landing logic in waypoint generator
  [x_ref, y_ref, z_ref] = waypointsHandle.get_waypoint([x, y, z, phi, theta, psi], landingFlag);
  
  [theta_d, phi_d, gTHR] = position_controller.control([x_ref, y_ref, z_ref], [x, y z, phi, theta, psi, u, v, w], dT);
  comm_thr_d = gTHR + T_trim;
  
  %{
        TODO: implement UI handler. Process commands like start, stop
  %}
  % positive pitch commands travel in negative x, so reverse it here 
  vehicle.control(comm_thr_d, phi_d, -theta_d, psi_d);
  % Sleep until next loop
  waitfor(rate_controller);
end

% Shut off drone
% TODO: shutdown vehicle
% vehicle.shutdown()

% Signal to UI that program is done
disp('Done');
