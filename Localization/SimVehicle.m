classdef SimVehicle <  Interfaces.Localizer & handle

    properties
        udp;
        host;
        port;
    end

    methods

        function success = shutdown(obj)
            obj.client.Uninitialize();
			success = true;
        end

        function obj = SimVehicle(host, port)

            obj.host = host;
            obj.port = port;
            obj.udp = udpport('Datagram', 'LocalPort', obj.port);
        end

        function connect(obj)
        end

        function disconnect(obj)
        end

        function [x, y, z, phi, theta, psi] = get_position(obj, id, filtered)
            if obj.udp.NumDatagramsAvailable > 0
                res = read(obj.udp, 1, 'double');
                % [p, q, r, phi, theta, psi, u, v, w, x, y, z] = res.Data;
                % disp([p, q, r, phi, theta, psi, u, v, w, x, y, z])
                if length(res) >= 1
                    data = typecast(res.Data(1:24), 'double');
                    [p, q, r, phi, theta, psi, u, v, w, x, y, z] = deal(data(1), data(2), data(3), data(4), data(5), data(6), data(7), data(8), data(9), data(10), data(11), data(12)) ;
                end
            end
        end

    end

end

