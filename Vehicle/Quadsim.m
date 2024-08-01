classdef Quadsim < Vehicle
    
    properties
        ip
        port
        udp
        udprx
    end
    
    methods (Static)
        function quadsim = from_yaml(yaml)
            % Add path to the YAML parser library (update the path accordingly)
            
            % Example of a YAML structure:
            % key1:
            %   - item1
            %   - item2
            % key2:
            %   - item3
            %   - item4
            
            % Specify the key you want to iterate over
            key = 'key1';
            
            % Check if the key exists in the YAML data
            if isfield(yaml, key)
                items = yamlData.(key);
                
                % Iterate over the items under the key
                for i = 1:length(items)
                    item = items{i};
                    fprintf('Item %d: %s\n', i, item);
                end
            else
                fprintf('Key "%s" not found in the YAML file.\n', key);
            end
            
        end
    end
    methods
        function obj = Quadsim(obj, ip, port)
            if nargin == 2
                obj.ip = ip;
                obj.port = port;
            else
              obj.ip = '127.0.0.1';
              obj.port = 25000;
            end
            
            % Define the remote IP and port
            obj.udp = udpport('datagram','IPV4');
            % obj.udprx = udpport('Datagram', 'LocalPort', 25001);
            
        end
        
        % todo: load initial conditions file
        function obj = initialize(obj)
        end
        
        % cyber system
        function mode = control_mode(obj)
        end
        
        % todo: this port is going to be used to mock the localization
        % interface, get this from estimated or something
        % get vehicle state 12 vector
        function [x, y, z, u, v, w, phi, theta, psi, p, q, r] = state(obj)
          if obj.udprx.NumDatagramsAvailable > 0
            res = read(obj.udprx, 1, 'double');
            % disp(res.Data)
            % [p, q, r, phi, theta, psi, u, v, w, x, y, z] = res.Data;
            % disp([p, q, r, phi, theta, psi, u, v, w, x, y, z])
            if length(res) >= 1
              data = typecast(res.Data(1:24), 'double');
              [p, q, r, phi, theta, psi, u, v, w, x, y, z] = deal(data(1), data(2), data(3), data(4), data(5), data(6), data(7), data(8), data(9), data(10), data(11), data(12)) ;
            end
            
          end
        end
        
        function control(obj, phi, theta, psi, thrust)
          % Define the data to be sent (4 values)
          data = [phi, theta, psi, thrust];
          dataToSend = typecast(double(data), 'uint8');
          write(obj.udp, dataToSend, 'uint8', obj.ip, obj.port);
        end
        
        function armed = arm(obj)
          armed = 1;
        end
        
        function disarmed = disarm(obj)
          disarmed=1;
        end
        
    end
end
