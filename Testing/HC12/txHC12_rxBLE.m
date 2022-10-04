clear;
clc;
device = serialport("COM5",115200)
flush(device)   
close all

% %% Connect to the Drone via Bluetooth
b = ble("C02824365530"); % Make sure to get the the Drone's MAC address before running this code
joy_c = characteristic(b, "00000000-0001-11E1-9AB4-0002A5D5C51B" , "00008000-0001-11E1-AC36-0002A5D5C51B") % Write w/out response
joy_c_imu = characteristic(b, "00000000-0001-11E1-9AB4-0002A5D5C51B" , "00E00000-0001-11E1-AC36-0002A5D5C51B") % Read IMU
% joy_c_imu.DataAvailableFcn = @displayCharacteristicData
% subscribe(joy_c_imu,'notification');

% Arm the drone: 
write(joy_c, [22, 128, 0, 128, 128, 0, 4], 'uint8', "WithoutResponse");
java.lang.Thread.sleep(2*1000); % Java sleep is much more accurate than matlab's pause (sleep in ms)


wTimes = [0];
rTimes = [];
startByte = 245;
endByte = 2;
val=0;
count = 0;
readLoop = tic;
global data;
global timestamps;
data = zeros(1000+1,20); % For reading IMU
timestamps = datetime(zeros(10,1), 0, 0); %a 10x1 array of datetime
disp("Starting loop")
global index;
index = 1;
j=1;


euler = [];
euler_rates = [];
for i=1:1000
    loop = tic;
    val = 30+mod(i,10);

    % Read data
    write(device,[245 val 0 val val 2],"uint8");
    [data(i,:), timestamps(i)] = read(joy_c_imu, 'latest');

    % Parse data
    [thx,thy,thz] = parse_ble_euler(data(i,3:8),10);
     euler(i,:) = [thx,thy,thz];
     [thx_rate,thy_rate,thz_rate] = parse_ble_euler(data(i,9:14),100);
     euler_rates(i,:) = [thx_rate,thy_rate,thz_rate];
    
%     read(joy_c_imu, 'latest');

    java.lang.Thread.sleep(7); % It takes like 100ms to switch from reading something to writing something (limitation of the hardware)
    t = toc(loop);
    wTimes(i) = t;
end



for i=1:50
    write(device,[245 0 0 0 0 2],"uint8")
end

figure()
plot(wTimes)

figure()
plot(euler)

figure()
plot(euler_rates)

% unsubscribe(joy_c_imu);
% clear joy_c_imu
% clear b

% 
% function displayCharacteristicData(src,evt)
%     global index;
%     global data;
%     global timestamps;
%     [d,t] = read(src, 'oldest');
% %     disp(d)
% %     disp(t)
%     data(index,:) = d;
%     timestamps(index) = t;
%     index=index+1;
% end


