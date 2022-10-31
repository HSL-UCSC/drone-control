classdef Communication < handle
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
        joy_c_imu;
        joy_c_arm;
    end 
    methods
        function init = init(obj, macAddress, comPort, baudRate)
            % Create bluetooth connection
            b = ble(macAddress); % ST DRONE FRAME 1
            obj.joy_c_imu = characteristic(b, "00000000-0001-11E1-9AB4-0002A5D5C51B" , "00E00000-0001-11E1-AC36-0002A5D5C51B") % Read IMU
            obj.joy_c_arm = characteristic(b, "00000000-0001-11E1-9AB4-0002A5D5C51B" , "00000400-0001-11E1-AC36-0002A5D5C51B") % Read arming status
            
            % Open serial port for HC12 connection
            device = serialport(comPort,baudRate)
            flush(device)
        end

        function [data, timestamps] = readImuBLE(obj)
            [data, timestamps] = read(obj.joy_c_imu, 'latest');
        end

        function [data, timestamps] = readArmBLE(obj)
            [data, timestamps] = read(obj.joy_c_arm, 'latest');
        end

        function [out] = parseBLE(data, scale) % Scale is for decimal precision (choose what was chosen at the firmware level)
            if(data(2) < 10)
                out = (256*data(2) + data(1))/scale;
            else
                out = -((256*(255 - data(2)) + (256-data(1))))/scale;
            end
        end

        function packet = attitudeCmdPacket(obj,yaw,thrust,roll,pitch)
            % Build attitude command packet
            packet = [obj.startByte, yaw, thrust, roll, pitch, obj.endByte];
        end

        function packet = dataUpdatePacket(obj,index,value)
            % Build attitude command packet
            packet = [obj.startByte, obj.startByte_DataUpdate, obj.endByte_DataUpdate, index, value, obj.endByte];
        end
        
    end
end