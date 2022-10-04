clear;
clc;
device = serialport("COM5",115200)
flush(device)
close all

% % %% Connect to the Drone via Bluetooth
% b = ble("C02824365530"); % Make sure to get the the Drone's MAC address before running this code
% joy_c = characteristic(b, "00000000-0001-11E1-9AB4-0002A5D5C51B" , "00008000-0001-11E1-AC36-0002A5D5C51B") % Write w/out response
% joy_c_imu = characteristic(b, "00000000-0001-11E1-9AB4-0002A5D5C51B" , "00E00000-0001-11E1-AC36-0002A5D5C51B") % Read IMU
% 
% % Arm the drone: 
% write(joy_c, [0, 0, 0, 0, 0, 0, 2], 'uint8', "WithoutResponse") % is code running on drone different? Should send a 4 to arm?
% java.lang.Thread.sleep(2*1000); % Java sleep is much more accurate than matlab's pause (sleep in ms)
% write(joy_c, [22, 128, 0, 128, 128, 0, 0], 'uint8', "WithoutResponse");

% 
% 
% while(1)
%     tic
%     read(device,1,"uint8")
%     toc
% %     if(device.NumBytesAvailable >= 1)
% %         read(device,1,"uint8")
% %     end
% end


wTimes = [0];
rTimes = [];
startByte = 245;
endByte = 2;
val=0;

count = 0;
readLoop = tic;
data = zeros(1000+1,20); % For reading IMU
timestamps = datetime(zeros(10,1), 0, 0); %a 10x1 array of datetime
disp("Starting loop")
i = 1;
loop = tic;
while(1)
    if(device.NumBytesAvailable >= 1)
        t = toc(loop);
        wTimes(i) = t;
        i=i+1;
        loop = tic;
        val = 20+mod(i,10);
        read(device,1,"uint8")
        write(device,[245 val 0 val val 2],"uint8");
%         [data(i,:), timestamps(i)] = read(joy_c_imu, 'latest');

    end
%     java.lang.Thread.sleep(30); % It takes like 100ms to switch from reading something to writing something (limitation of the hardware)

%     if(device.NumBytesAvailable >= 1)
%         count = count + 1;
%         read(device,1,"uint8")
%         r = toc(readLoop);
%         rTimes(i) = r;
%         readLoop = tic;
%     end
end

for i=1:50
    write(device,[245 0 0 0 0 2],"uint8")
end

plot(wTimes)






% count = 0;
% tic
% while(1)
%     if(device.NumBytesAvailable >= 1)
%         read(device,1,"uint8")
%         t = toc
%         tic
%     end
% end

% j=1;
% rTimes = [];
% valsRead = 0;
% numSent = 0;
% while(1)
%     if(j<100)
%         write(device,[3 22 22 22 22 249],"uint8")
%     else
%         write(device,[3 22 22 22 22 244],"uint8")
%     end
%     pause(0.1)
%     if(device.NumBytesAvailable >= 4)
%         tic
%         valsRead = read(device,4,"uint8")
%         char(valsRead)
%         t = toc;
%         rTimes(j) = t;
%         j = j+1;
%     end
%     numSent = numSent + 1;
% end

% S = char(vals)
% 
% function x = instrcallback()
%     tic
%     read(device,4,"uint8")
%     t = toc
% end

% plot(wTimes*1000)
% clear
