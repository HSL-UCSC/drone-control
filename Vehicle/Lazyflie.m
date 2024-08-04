classdef Lazyflie < Interfaces.Multirotor

    properties
        control_mode string = "manual";
        initialized boolean = false;
        armed boolean = false;
        control_writer Interfaces.Writer = nil;
        state_reader Interfaces.Reader = nil;
    end

    methods

        function obj = Lazyflie(ble_address, com_port, baud_rate)

            arguments
                ble_address string
                com_port string
                baud_rate int = 38400;
            end

            obj.state_reader = BLEReader(ble_address);
            obj.control_writer = HC12Writer(com_port, baud_rate);

        end

        function initialize(obj)

            for loop_index = 1:inf
                % Check if status onboard is armed

                if obj.is_armed() == 1
                    break;
                end

                % todo: initialize not implemented. CG needs RC handle for this initialization loop to make sense
                return;
                % Check xbox input and send arm command when 'RB' is pressed
                prevCalibrate = calibrate; prevArm = arm; prevOverride = override; prevLand = land; prevSetpointMode = setpointMode; prevSetpointPrev = setpointPrev; prevSetpointNext = setpointNext;
                [xbox_comm_thrust, xbox_comm_yaw, xbox_comm_pitch, xbox_comm_roll, calibrate, arm, override, land, ...
                     setpointMode, setpointPrev, setpointNext] = xboxControllerHandle.getState();

                if (arm && ~prevArm)
                    disp("Arming")
                    obj.arm();
                end

                if (calibrate && ~prevCalibrate)
                    disp("Calibrating")
                    obj.calibrate();
                end

                pause(0.1);
            end

            % Initialize the lazyflie
            obj.initialized = true;
        end

        function armed = arm(obj)
            % set arming status

            % read arming status
            armed = obj.is_armed();

            if (armed == 1)
                obj.armed = true;
                return
            end

            % todo: check for error condition
            obj.control_writer.write(obj.dataUpdatePacket(obj.ARM, 1));
            % if no error, set armed to true
            obj.armed = true;
        end

        function armed = is_armed(obj)
            % read arming status
            armed = lazyFlie.state_reader.read(ble_arm_char);
            return;
        end

        function calibrate(obj)
            % calibrate the lazyflie
            obj.control_writer.write(obj.dataUpdatePacket(obj.CALIBRATION, 1));
        end

        % TODO: read doesn't return timestamps, best I can tell
        function [data, timestamps] = read(obj, characteristic)

            if ~obj.inititialized
                return
            end

            [data, timestamps] = read(characteristic, 'latest');
        end

        function packet = write(obj, packet)
            write(obj.serial_device, packet, "uint8")
        end

        function packet = attitudePacket(obj, yaw, thrust, roll, pitch)
            % Build attitude command packet
            packet = [obj.startByte, yaw, thrust, roll, pitch, obj.endByte];
        end

        function packet = dataUpdatePacket(obj, index, value)
            % Build attitude command packet
            packet = [obj.startByte, obj.startByte_DataUpdate, obj.endByte_DataUpdate, index, value, 0, 0, 0, 0, 0, 0, 0, 0, 0, obj.endByte];
        end

        function packet = geometricAttitudePacket(obj, thrust, R_d, Omega_d)
            % Build geometric attitude command packet
            % Map -1,1 to 0,255
            Rcmd = zeros(3, 3);
            OmegaCmd = zeros(3);

            % 1 byte
            MAX = 1;
            slope_m = 255.0 / (MAX - -MAX);
            Rcmd = uint8(slope_m * (R_d + MAX));

            % % 2 bytes
            % MAX = 1;
            % slope_m = 65535.0/(MAX - -MAX); %255
            % Rcmd = slope_m *(R_d + MAX);
            % Rcmd_row1 = typecast(uint16(Rcmd(1,:)),'uint8');
            % Rcmd_row2 = typecast(uint16(Rcmd(2,:)),'uint8');
            % Rcmd_row3 = typecast(uint16(Rcmd(3,:)),'uint8');

            MAX = 0.5;
            slope_m = 255.0 / (MAX - -MAX);
            OmegaCmd = uint8(slope_m * (Omega_d + MAX));

            packet = [obj.startByte, uint8(thrust), Rcmd(1, 1), Rcmd(1, 2), Rcmd(1, 3), Rcmd(2, 1), Rcmd(2, 2), Rcmd(2, 3), Rcmd(3, 1), Rcmd(3, 2), Rcmd(3, 3), OmegaCmd(1), OmegaCmd(2), OmegaCmd(3), obj.endByte];
        end

        % TODO: get rc controller input, autopilot input as arguments
        function control(obj, comm_thr_d, comm_phi_d, comm_theta_d, comm_yaw_d, MAX_ANGLE)
            arguments
                obj
                comm_thr_d (1, 1) double
                comm_phi_d (1, 1) double
                comm_theta_d (1, 1) double
                comm_yaw_d (1, 1) double
                MAX_ANGLE = 12.0
            end

            comm_phi_d = min(max(-MAX_ANGLE, comm_phi_d), MAX_ANGLE);
            comm_theta_d = min(max(-MAX_ANGLE, comm_theta_d), MAX_ANGLE);
            attitude_command_packet = obj.attitudePacket(comm_yaw_d, comm_thr_d, comm_phi_d, comm_theta_d);
            obj.control_writer.write(attitude_command_packet);

        end

        % TODO: necessary to clear?
        function shutdown()
            disp("Shutting Down...")

            unsubscribe(obj.ble_imu_char)
            clear obj.ble_imu_char;

            for ik = 1:10
                communicationsHandle.sendAttitudeCmdPacket(device, 128, 0, 128, 128);
                pause(0.05)
            end

            % Shutdown cleanup
            obj.localization.shutdown();
            obj.communicationsHandle.shutdown()

            disp("Shutting Down...")
        end

    end

end
