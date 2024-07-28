% todo: import/user cybertx matlab example
classdef CyberTXWriter < Interfaces.Writer & handle

    properties
        client
    end

    methods

        function obj = CyberTXWriter(com_port, baud_rate, num_channels)
            obj.client = CyberTX(com_port, baud_rate, num_channels);
        end

        % todo: double check against CyberTX matlab
        function packet = write(obj, packet)
            obj.client.writePPM(packet);
        end

    end

    methods(static)
        function obj = Command(aileron, elevator, throttle, rudder, arm, mode)
            obj = struct('Aileron', aileron, 'Elevator', elevator, 'Throttle', throttle, 'Rudder', rudder, 'Arm', arm, 'Mode', mode);
        end
    end

end
