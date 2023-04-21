classdef Communications < handle
    properties
        % Packet structure variables
        startByte = 245; 
        endByte = 2;
        startByte_DataUpdate = 5;
        endByte_DataUpdate = 10;

        % Packet commands
        DR_UPDATE_ARM = 1;
        DR_UPDATE_CAL = 2;
        DR_UPDATE_CM = 3;

        % BLE characteristic
%         ble_imu_char = characteristic(b, "00000000-0001-11E1-9AB4-0002A5D5C51B" , "00E00000-0001-11E1-AC36-0002A5D5C51B"); % Read IMU
%         ble_arm_char = characteristic(b, "00000000-0001-11E1-9AB4-0002A5D5C51B" , "20000000-0001-11E1-AC36-0002A5D5C51B"); % Read arming status
%         ble_bat_char = characteristic(b, "00000000-0001-11E1-9AB4-0002A5D5C51B" , "001D0000-0001-11E1-AC36-0002A5D5C51B"); % Read battery/pressure status
    end 
    methods

        function [data, timestamps] = readBLE(obj, ble_char)
            [data, timestamps] = read(ble_char, 'latest');
        end


        function [out] = parseBLE(obj, data, scale) % Scale is for decimal precision (choose what was chosen at the firmware level)
            if(data(2) < 128)
                out = (256*data(2) + data(1))/scale;
            else
                out = -((256*(255 - data(2)) + (256-data(1))))/scale;
            end
        end

        function packet = sendAttitudeCmdPacket(obj,device,yaw,thrust,roll,pitch)
            % Build attitude command packet
            packet = [obj.startByte, yaw, thrust, roll, pitch, obj.endByte];
            write(device,packet,"uint8") 
        end

        function packet = sendGeometricAttitudeCmdPacket(obj,device,thrust,R_d,Omega_d)
            % Build geometric attitude command packet
            % Map -1,1 to 0,255
            Rcmd = zeros(3,3);
            OmegaCmd = zeros(3);
            
            % 2 bytes
%             MAX = 1;
%             slope_m = 65535.0/(MAX - -MAX); %255
%             Rcmd = slope_m *(R_d + MAX);
%             Rcmd_row1 = typecast(uint16(Rcmd(1,:)),'uint8');
%             Rcmd_row2 = typecast(uint16(Rcmd(2,:)),'uint8');
%             Rcmd_row3 = typecast(uint16(Rcmd(3,:)),'uint8');

            % 1 byte
            MAX = 1;
            slope_m = 255.0/(MAX - -MAX);
            Rcmd = uint8(slope_m *(R_d + MAX));

            MAX = 4;
            slope_m = 255.0/(MAX - -MAX);
            OmegaCmd = uint8(slope_m *(Omega_d + MAX));

            packet = [obj.startByte, uint8(thrust), Rcmd(1,1), Rcmd(1,2), Rcmd(1,3), Rcmd(2,1), Rcmd(2,2), Rcmd(2,3), Rcmd(3,1), Rcmd(3,2), Rcmd(3,3), OmegaCmd(1), OmegaCmd(2), OmegaCmd(3), obj.endByte];
%             packet = [obj.startByte, uint8(thrust), Rcmd_row1(1), Rcmd_row1(2), Rcmd_row1(3), Rcmd_row1(4), Rcmd_row1(5), Rcmd_row1(6), Rcmd_row2(1), Rcmd_row2(2), Rcmd_row2(3), Rcmd_row2(4), Rcmd_row2(5), Rcmd_row2(6), Rcmd_row3(1), Rcmd_row3(2), Rcmd_row3(3), Rcmd_row3(4), Rcmd_row3(5), Rcmd_row3(6), OmegaCmd(1), OmegaCmd(2), OmegaCmd(3), obj.endByte];
            write(device,packet,"uint8") 
        end

        function packet = sendDataUpdatePacket(obj,device,index,value)
            % Build attitude command packet
            packet = [obj.startByte, obj.startByte_DataUpdate, obj.endByte_DataUpdate, index, value, 0, 0, 0, 0, 0, 0, 0, 0, 0, obj.endByte];
%             packet = [obj.startByte, obj.startByte_DataUpdate, obj.endByte_DataUpdate, index, value, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, obj.endByte];
            write(device,packet,"uint8")
        end

        function packet = sendDummyPacket(obj,device)
            % Build attitude command packet
            packet = [1, 1, 1, 1, 1, 1];
            write(device,packet,"uint8") 
        end
        
    end
end


%             for i=1:3
%                 for j=1:3
%                     Rcmd(i,j) = uint8(slope_m *(R_d(i,j) + MAX));
%                 end
%             end