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
waypointsHandle = Path.EventPath([0 0 1]);

control_frequency_hz = 120;
dT = 1 / control_frequency_hz;


motion_capture.initialize();
disp("Motion capture interface initialized")

% waypoints handle is a concrete implementation of the Path interface
MAX_ANGLE = 12.0;

position_controller = Control.VelocityController(control_frequency_hz, 10);

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

T_trim = .65;
roll_trim = 0;
pitch_trim = 0;

vehicle_state = motion_capture.get_pose(vehicle.id);
p = vehicle_state.translation;
R = vehicle_state.rotation;
[xprev, yprev, zprev, phiprev, thetaprev, psiprev] = deal(p(1), p(2), p(3), R(1), R(2), R(3));

% configure control loop rate
rate_controller = rateControl(control_frequency_hz);
waitfor(rate_controller);
gTHR = 0;

vh = [0, 0, 0];
ph = [0, 0, 0];
ah = [0, 0, 0];
as = [0, 0, 0];

while 1
  
  dT = rate_controller.LastPeriod;
  
  % todo: less jank way to get arming confirm
  if vehicle.armed == 0
    res = input('');
    disp(res);
    vehicle.armed = 1;
  end
  
  % Get new drone position and store
  vehicle_state = motion_capture.get_pose(vehicle.id);
  p = vehicle_state.translation / 1000;
  R = vehicle_state.rotation;
  v = vehicle_state.velocity / 1000;
  
  [x, y, z, phi, theta, psi, u, v, w] = deal(p(1), -p(2), p(3), R(1), R(2), R(3), v(1), -v(2), v(3));
  ph = [ph; [x, y, z]];
  ub = (u * cos(psi) + v * sin(psi));
  vb = (v * cos(psi) - u * sin(psi));
  vh = [vh; [ub, vb, w]];

  % TODO: implement landing logic in waypoint generator
  [x_ref, y_ref, z_ref] = waypointsHandle.get_waypoint([x, y, z, phi, theta, psi], landingFlag);
  
  [theta_d, phi_d, gTHR] = position_controller.control([x_ref, y_ref, z_ref], [x, y z, phi, theta, psi, u, v, w], dT);
  % TODO: add feedforward term to PIDs for trim
  % gTHR = gTHR + T_trim;
  
  %{
        TODO: implement UI handler. Process commands like start, stop
  %}
  % positive pitch commands travel in negative x, so reverse it here

  vehicle.control(gTHR, phi_d, theta_d, psi_d);
  as = [as; [phi, theta, psi]];
  ah = [ah; [phi_d, -theta_d, psi_d]];
  % Sleep until next loop
  waitfor(rate_controller);
end

% Shut off drone
% TODO: shutdown vehicle
% vehicle.shutdown()

% Signal to UI that program is done
disp('Done');
