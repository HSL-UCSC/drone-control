% todo: import/user cybertx matlab example
classdef CyberTXWriter < Interfaces.Writer & handle

    properties
        client
    end

    methods

        function obj = CyberTXWriter(com_port, baud_rate, line, num_channels)
            obj.client = CyberTX(com_port, baud_rate, line, num_channels);
        end

        function command = write(obj, command)
            obj.client.writePPM([command.Aileron, command.Elevator, command.Throttle, command.Rudder]);
        end
    end

    methods(Static)
        function command = Command(aileron, elevator, throttle, rudder, arm, mode)
            arguments
                aileron
                elevator
                throttle
                rudder
                arm = 0
                mode = 0
            end

            aileron = max(1000, min(2000, cast(aileron, 'uint16')));
            elevator = max(1000, min(2000, cast(elevator, 'uint16')));
            throttle = max(1000, min(2000, cast(throttle, 'uint16')));
            rudder = max(1000, min(2000, cast(rudder, 'uint16')));

            % todo: nargin check for arm, mode
            % command = struct('Aileron', aileron, 'Elevator', elevator, 'Throttle', throttle, 'Rudder', rudder, 'Arm', arm, 'Mode', mode);
            command = struct('Aileron', aileron, 'Elevator', elevator, 'Throttle', throttle, 'Rudder', rudder);

        end
    end

end
