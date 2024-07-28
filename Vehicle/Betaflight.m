classdef Betaflight < Interfaces.Multirotor

    properties
        control_mode string = "manual";
        initialized boolean = false;
        armed boolean = false;
        control_writer Interfaces.Writer = nil;
    end

    methods

        function obj = Betaflight(com_port, baud_rate, line, channels)

            arguments
                com_port string
                baud_rate int = 115200;
                line int = 1;
                channels int = 8;
            end

            % TODO: handle case of non-nil reader/writer to support dependency injection
            obj.control_writer = CyberTXWriter(com_port, baud_rate, 1, 8);

        end

        function packet = write(obj, packet)
            write(obj.serial_device, packet, "uint8")
        end

        function packet = attitudePacket(obj, yaw, thrust, roll, pitch)
            % Build attitude command packet
            packet = [obj.startByte, yaw, thrust, roll, pitch, obj.endByte];
        end

        % TODO: get rc controller input, autopilot input as arguments
        function control(obj, controlMode, comm_yaw_d, comm_thr_d, comm_phi_d, comm_theta_d)

            % Send attitude command
            % todo: RC interface
            if (controlMode == 0)
                attitude_command_packet = obj.attitudePacket(xbox_comm_yaw, xbox_comm_thrust, xbox_comm_roll, xbox_comm_pitch)
                obj.write(attitude_command_packet);
            else
                attitude_command_packet = obj.attitudePacket(comm_yaw_d, comm_thr_d, comm_phi_d, comm_theta_d);
                obj.control_writer.write(attitude_command_packet);
            end

        end

    end

end
