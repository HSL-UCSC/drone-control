classdef Betaflight < Interfaces.Multirotor

    properties
        control_mode = "manual";
        initialized = false;
        armed = false;
        control_writer;
    end

    methods

        function obj = Betaflight(com_port, baud_rate, line, channels)

            arguments
                com_port
                baud_rate = 115200;
                line = 1;
                channels = 4;
            end

            % TODO: handle case of non-nil reader/writer to support dependency injection
            obj.control_writer = CyberTXWriter(com_port, baud_rate, 1, 4);

        end

        function packet = write(obj, packet)
            write(obj.serial_device, packet, "uint8");
        end

        function packet = attitudePacket(obj, thrust, roll, pitch, yaw)
            % Build attitude command packet
            packet = [yaw, thrust, roll, pitch];
        end

        function arm(obj)
        end

        function disarm(obj)
        end


        % TODO: get rc controller input, autopilot input as arguments
        function control(obj, comm_thr_d, comm_phi_d, comm_theta_d, comm_yaw_d)
            attitude_command_packet = obj.control_writer.Command(comm_thr_d, comm_phi_d, comm_theta_d, comm_yaw_d);
            obj.control_writer.write(attitude_command_packet);
        end

    end

end
