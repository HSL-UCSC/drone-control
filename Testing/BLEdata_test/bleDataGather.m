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
b = ble("C02874363E30"); % Make sure to get the the Drone's MAC address before running this code
char = b.Characteristics; % Get the characteristics of the Drone

%% Assign the rigid body id. Double check with motive that the rigid body ID is correct.
Drone_ID = 1;

%% Store the reference to the Charactersitic responsible for Writing Joystick data:
% https://www.st.com/resource/en/user_manual/dm00550659-getting-started-with-the-bluest-protocol-and-sdk-stmicroelectronics.pdf

joy_c_imu = characteristic(b, "00000000-0001-11E1-9AB4-0002A5D5C51B" , "00E00000-0001-11E1-AC36-0002A5D5C51B") % Read IMU
joy_c_pressure = characteristic(b, "00000000-0001-11E1-9AB4-0002A5D5C51B" , "001D0000-0001-11E1-AC36-0002A5D5C51B") % Read Pressure Sensor and battery voltage
joy_c_arming = characteristic(b, "00000000-0001-11E1-9AB4-0002A5D5C51B" , "20000000-0001-11E1-AC36-0002A5D5C51B") % Read Arming signal

joy_c_write = characteristic(b, "00000000-0001-11E1-9AB4-0002A5D5C51B" , "00008000-0001-11E1-AC36-0002A5D5C51B") % Write w/out response
%% Next calibrate/arm the drone
% Reference slides 25,26 from - https://www.st.com/content/ccc/resource/sales_and_marketing/presentation/product_presentation/group0/bd/cc/11/15/14/d4/4a/85/STEVAL-DRONE01_GETTING_STARTED_GUIDE/files/steval-drone01_getting_started_guide.pdf/jcr:content/translations/en.steval-drone01_getting_started_guide.pdf

% 1) Place drone down flat and press reset button to calibrate it
% 2) Arm the drone: 
write(joy_c_write, [0, 0, 0, 0, 0, 0, 2], 'uint8', "WithoutResponse") % is code running on drone different? Should send a 4 to arm?
java.lang.Thread.sleep(2*1000); % Java sleep is much more accurate than matlab's pause (sleep in ms)
write(joy_c_write, [22, 128, 0, 128, 128, 0, 0], 'uint8', "WithoutResponse");



%% Keep track of previous position for computing velocity.

java.lang.Thread.sleep(5*1000);
disp("STARTING")

data = zeros(100,20);
timestamps = datetime(zeros(10,1), 0, 0); %a 10x1 array of datetime
writeTimes = zeros(100,1);
readTimes = zeros(100,1);
sleepTimes = zeros(100,1);
getPosTimes = zeros(100,1);
loopTimes = zeros(100,1);
Drone_pos_data = [];
Drone_rate_data = [];
dTlist = [];
thr=50; % from 100 to 200
ail = 0;

[prevDronePos] = GetDronePosition(theClient, Drone_ID);
dT = 0.028; % Initial dT estimate (changes on every loop)>
euler = zeros(100,3);
euler_rates = zeros(100,3);
for i=1:1000
    loopT = tic;
    

    if(ail==250)
        ail=0
    end
    ail=ail+1;
    
    writeTime = tic;
    write(joy_c_write, [0, ail, thr, 128, 128, 0, 5], 'uint8', "WithoutResponse")
    writeTimes(i) = toc(writeTime);

    
    
    
    % Read current position from drone (~15ms off max) (7ms for read operation and getting
    % 2nd buffered piece of data so its ~6ms off due to that (160Hz))(when
    % I write a command I get it back twice and don't get the last thing I 
    % sent at the very end)
    
    readTime = tic;
    [data(i,:), timestamps(i)] = read(joy_c_imu, 'latest');
    readTimes(i) = toc(readTime);
    
    [thx,thy,thz] = parse_ble_euler(data(i,3:8),10);
    euler(i,:) = [thx,thy,thz];
    [thx_rate,thy_rate,thz_rate] = parse_ble_euler(data(i,9:14),100);
    euler_rates(i,:) = [thx_rate,thy_rate,thz_rate];
%     euler is ~15ms behind at this point
    
    
    % Read current position from Motive (~5ms off max) (~4ms behind due to latency and ~1.5
    % ms behind from function call)
    getPosTime = tic;
    [DronePos] = GetDronePosition(theClient, Drone_ID);
    getPosTimes(i) = toc(getPosTime);
    Drone_pos_data(i,:) = DronePos;
    Drone_rate_data(i,:) = (DronePos - prevDronePos)/dT;
    prevDronePos = DronePos;
    % euler is ~5ms behind at this point
    

    % Sleep 15ms
%     sleepTime = tic;
%     java.lang.Thread.sleep(30);
%     sleepTimes(i) = toc(sleepTime);
    
    dT =  toc(loopT);
    loopTimes(i) = dT;
end


%% Disconnect from drone and motive
ik = 0;
while ik < 10
    write(joy_c_write, [0, 128, 0, 128, 128, 0, 0], 'uint8', "WithoutResponse");
    java.lang.Thread.sleep(100);
    ik = ik+1;
end
theClient.Uninitialize();
disp('Done')

%% The position from Motive should be ~10ms ahead of drone position
% Drone assumed to have arrow facing away from desktop station
close all;




% Find timeshift (and corresponding time delay) through xcorr (time delay should be 10-15ms (ie. = read() time?))
[c,lags] = xcorr(euler(:,1),Drone_pos_data(:,1)*(180/pi));
timeShift = find(c==max(c)) - 1000; % This number corresponds to how many ms?
RMSE_pitch_angle = sqrt(mean((euler(1+timeShift:end,1) - Drone_pos_data(1:end-timeShift,1)*(180/pi)).^2))  % Root Mean Squared Error
% Plot with timeshift
figure();
plot(euler(1+timeShift:end,1));
hold on;
plot(Drone_pos_data(1:end-timeShift,1)*(180/pi))
legend("drone thx-pitch","motive thx-pitch");


% Find timeshift
[c,lags] = xcorr(euler(:,2),Drone_pos_data(:,2)*(180/pi));
timeShift = find(c==max(c)) - 1000; % This number corresponds to how many ms?
RMSE_roll_angle = sqrt(mean((euler(1+timeShift:end,2) - Drone_pos_data(1:end-timeShift,2)*(180/pi)).^2))  % Root Mean Squared Error
% Plot with timeshift
figure();
plot(euler(1+timeShift:end,2));
hold on;
plot(Drone_pos_data(1:end-timeShift,2)*180/pi)
legend("drone thy-roll","motive thy-roll");


pitchRatesFil_motive = lowpass(euler_rates(:,1)*180/pi,0.1);
pitchRatesFil_drone = lowpass(Drone_rate_data(:,1)*180/pi,0.1);
figure();
plot(pitchRatesFil_motive);
hold on;
plot(pitchRatesFil_drone)
legend("drone thx-pitch rate","motive thx-pitch rate");

rollRatesFil_motive = lowpass(euler_rates(:,2)*180/pi,0.1);
rollRatesFil_drone = lowpass(Drone_rate_data(:,2)*180/pi,0.1);
figure();
plot(rollRatesFil_motive);
hold on;
plot(rollRatesFil_drone)
legend("drone thy-roll rate","motive thy-roll rate");
