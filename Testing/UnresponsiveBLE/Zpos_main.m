%% Clear past data
close all;  
clear all;
clc;

%% Instantiate client object to run Motive API commands
% https://optitrack.com/software/natnet-sdk/

% Create Motive client object
% dllPath = fullfile('d:','DroneControl','NatNetSDK','lib','x64','NatNetML.dll');
% assemblyInfo = NET.addAssembly(dllPath); % Add API function calls
% theClient = NatNetML.NatNetClientML(0);
% 
% % Create connection to localhost, data is now being streamed through client object
% HostIP = '127.0.0.1';
% theClient.Initialize(HostIP, HostIP); 

%% Connect to the Drone via Bluetooth
b = ble("C0285B324333"); % Make sure to get the the Drone's MAC address before running this code
char = b.Characteristics; % Get the characteristics of the Drone

%% Assign the rigid body id. Double check with motive that the rigid body ID is correct.
Drone_ID = 1;

%% Store the reference to the Charactersitic responsible for Writing Joystick data:
% https://www.st.com/resource/en/user_manual/dm00550659-getting-started-with-the-bluest-protocol-and-sdk-stmicroelectronics.pdf

joy_c = characteristic(b, "00000000-0001-11E1-9AB4-0002A5D5C51B" , "00E00000-0001-11E1-AC36-0002A5D5C51B") % Read IMU

% joy_c_write = characteristic(b, "00000000-0001-11E1-9AB4-0002A5D5C51B" , "00008000-0001-11E1-AC36-0002A5D5C51B") % Write w/out response
%% Next calibrate/arm the drone
% Reference slides 25,26 from - https://www.st.com/content/ccc/resource/sales_and_marketing/presentation/product_presentation/group0/bd/cc/11/15/14/d4/4a/85/STEVAL-DRONE01_GETTING_STARTED_GUIDE/files/steval-drone01_getting_started_guide.pdf/jcr:content/translations/en.steval-drone01_getting_started_guide.pdf

% 1) Place drone down flat and press reset button to calibrate it
% 2) Arm the drone: 
% write(joy_c_write, [0, 0, 0, 0, 0, 0, 2], 'uint8', "WithoutResponse") % is code running on drone different? Should send a 4 to arm?
% java.lang.Thread.sleep(2*1000); % Java sleep is much more accurate than matlab's pause (sleep in ms)
% write(joy_c_write, [22, 128, 0, 128, 128, 0, 0], 'uint8', "WithoutResponse");


%% Keep track of previous position for computing velocity.

java.lang.Thread.sleep(5*1000);
disp("STARTING")

data = zeros(10,20);
timestamps = datetime(zeros(10,1), 0, 0); %a 10x1 array of datetime
for i=1:10
    [data(i,:), timestamps(i)] = read(joy_c);
    java.lang.Thread.sleep(1000);
end

% ik = 0;
% while ik < 10
%     write(joy_c_write, [0, 128, 0, 128, 128, 0, 0], 'uint8', "WithoutResponse");
%     java.lang.Thread.sleep(100);
%     ik = ik+1;
% end

% Finally, Close the Motive Client
% theClient.Uninitialize();

disp('Done')
%end


