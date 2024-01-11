classdef OptitrackNatNet <  Interfaces.Localizer & handle

    properties
        client;
        drone_id;
        filter Filter;
        last_x;
        last_y;
        last_z;
        lpf
    end

    methods

        function success = shutdown(obj)
            obj.client.Uninitialize();
			success = true;
        end

        function obj = Optitrack(dllPath, host_ip, drone_id, filter, n_warmup)

            arguments
                dllPath string
                host_ip string = '127.0.0.1'
                drone_id int8 = 1
                filter Filter.Interfaces.Filter = nil;
                n_warmup int = 250;
            end

            % https://optitrack.com/software/natnet-sdk/

            % Create Motive client object
            assemblyInfo = NET.addAssembly(dllPath); % Add API function calls
            obj.client = NatNetML.NatNetClientML(0);

            % Create connection to localhost, data is now being streamed through client object
            obj.client.Initialize(host_ip, host_ip);

            % TODO: use as default in get_position? Else, consider removal
            % Assign drone ID
            obj.drone_id = drone_id;
            obj.filter = filter;
            obj.warmup(obj.drone_id, n_warmup)

        end

        function warmup(obj, drone_id, n_warmup)

            arguments
                obj
                drone_id int8 = obj.drone_id
                n_warmup int16 = obj.n_warmup;
            end

            [drone_position] = obj.get_position(drone_id);
            obj.lpf.x = Filter.lpf_2_init(OUT_FREQ, CUT_OFF_FREQ_POS, drone_position(2));
            obj.lpf.y = Filter.lpf_2_init(OUT_FREQ, CUT_OFF_FREQ_POS, drone_position(3));
            obj.lpf.z = Filter.lpf_2_init(OUT_FREQ, CUT_OFF_FREQ_POS, drone_position(4));
            obj.lpf.vx = Filter.lpf_2_init(OUT_FREQ, CUT_OFF_FREQ_VEL, 0);
            obj.lpf.vy = Filter.lpf_2_init(OUT_FREQ, CUT_OFF_FREQ_VEL, 0);
            obj.lpf.vz = Filter.lpf_2_init(OUT_FREQ, CUT_OFF_FREQ_VEL, 0);

            position = obj.get_position(drone_id); 
            prev_t = position(1);
            prev_x = obj.lpf.x.filter(position(2));
            prev_y = obj.lpf.y.filter(position(3));
            prev_z = obj.lpf.z(position(4));
            
            for i = 1:n_warmup
                position = motionCaptureHandle.get_position();
            
                t = position(1);
                x_f = obj.lpf.x.filter(position(2));
                y_f = obj.lpf.y.filter(position(3));
                z_f = obj.lpf.z(position(4));
                
                dt = t - prev_t;
                obj.lpf.vx.filter((x_f - prev_x)/dt);
                obj.lpf.vy.filter((y_f - prev_y)/dt);
                obj.lpf.vz.filter((z_f - prev_z)/dt);

                prev_t = t; 
                prev_x = x_f;
                prev_y = y_f;
                prev_z = z_f;
            end

        end

        % TODO:
        function [time_stamp, x, y, z, pitch, roll, yaw] = get_position(obj, drone_id, filtered)

            arguments
                obj
                drone_id int8 = obj.drone_id
                filtered bool = false
            end

            % This function will get the data from Motive and extract the x, y, z
            % coordinates from the incoming data.

            R_x = [1 0 0; 0 cosd(-90) -sind(-90); 0 sind(-90) cosd(-90)];

            R_z = [cosd(90) -sind(90) 0; sind(90) cosd(90) 0; 0 0 1];
            frameData = obj.client.GetLastFrameOfData();

            %Get the time stamp
            time_stamp = frameData.fTimestamp;

            %Get the marker data

            drone_pos = frameData.RigidBodies(drone_id);                
            x = 0;
            y = 0;
            z = 0;
            yaw = 0;
         
            if length(drone_pos) > 0

                %% Motive coordiate frame
                %        ^ z
                %        |
                %        |
                % x <----O y(pointing up)
                %

                %% Our coordiate frame
                %
                % x <----O z(pointing up)
                %        |
                %        |
                %        v y

                %p = [drone_pos.x; drone_pos.y; drone_pos.z];

                %p_new = R_x * (R_z * p);
                %     x_d = -p(3);
                %     y_d = -p(1);
                %     z_d = p(2);

                x = drone_pos.x;
                y = -drone_pos.z;
                z = drone_pos.y;

                q = [drone_pos.qx, drone_pos.qy, drone_pos.qz, drone_pos.qw];
                Eul_ang = quat2eul(q);

                if Eul_ang(1) <= 0
                    pitch = pi + Eul_ang(1);
                else
                    pitch = Eul_ang(1) - pi;
                end
                yaw = -Eul_ang(2);
                roll = Eul_ang(3);
            end

            if filtered
                [time_stamp, x, y, z, pitch, roll, yaw] = obj.filter.filter([time_stamp, x_d, y_d, z_d, pitch, roll, yaw]);
            end

        end

    end

end