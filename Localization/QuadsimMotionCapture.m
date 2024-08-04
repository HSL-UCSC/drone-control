classdef QuadsimMotionCapture <  Interfaces.Localizer & handle

    properties
        vehicles;
    end

    methods



        function obj = QuadsimMotionCapture(vehicles)
            obj.vehicles = dictionary();
            for i = 1:length(vehicles)
                obj.vehicles(vehicles(i).id) = vehicles(i);
            end
        end

        function initialize(obj)
        end
        
        function success = shutdown(obj)
            obj.client.Uninitialize();
			success = true;
        end

        function register_vehicle(obj, vehicle)
            obj.vehicles(vehicle.id) = vehicle;
        end

        function pose = get_pose(obj, subject, segment, filtered)
            arguments
                obj
                subject
                segment = "";
                filtered = false
            end
            udp = obj.vehicles(subject).udprx;
            pose.translation = udp.UserData.translation;
            pose.rotation = udp.UserData.rotation;
            % if udp.NumDatagramsAvailable > 0
            %     res = read(udp, 1, 'double');
            %     % [p, q, r, phi, theta, psi, u, v, w, x, y, z] = res.Data;
            %     % disp([p, q, r, phi, theta, psi, u, v, w, x, y, z])
            %     if length(res) >= 1
            %         data = typecast(res.Data(1:12), 'double');
            %         pose.translation = {data(10), data(11), data(12)};
            %         pose.rotation = {data(4), data(5), data(6)};
            %         % [p, q, r, phi, theta, psi, u, v, w, x, y, z] = deal(data(1), data(2), data(3), data(4), data(5), data(6), data(7), data(8), data(9), data(10), data(11), data(12)) ;
            %     end
            % end

        end

    end

end

