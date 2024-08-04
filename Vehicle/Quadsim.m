classdef Quadsim < Interfaces.Multirotor
  
  properties
    mass_kg = .1;
    armed=1;
    control_mode="manual"
    id
    ip
    port
    udp
    udprx
  end
  
  methods (Static)
    function quadsim = from_dict(dict)
      if isfield(dict, 'ip')
        ip = dict.ip;
      else
        ip = '127.0.0.1';
      end
      if isfield(dict, 'port')
        port = dict.port;
      else
        port = 25000;
      end
      if isfield(dict, 'id')
        id = dict.id;
      else
        id = 25000;
      end
      quadsim = Quadsim(ip, port, id);
    end
  end
  methods
    function obj = Quadsim(ip, port, id)
      arguments
        ip = '127.0.0.1'
        port = 25000
        id = ''
      end
      obj.ip = ip;
      obj.port = port;
      obj.id = id;
      
      % Define the remote IP and port
      obj.udp = udpport('datagram','IPV4');
      obj.udprx = udpport('datagram', 'LocalPort', obj.port + 1);
      obj.udprx.ReadAsyncMode = 'continuous';
      obj.udprx.DatagramReceivedFcn = @obj.udp_handler;
    end
    
    function udp_handler(obj, src, event)
        if src.NumDatagramsAvailable > 0
            res = read(src, 1, 'double');
            % [p, q, r, phi, theta, psi, u, v, w, x, y, z] = res.Data;
            % disp([p, q, r, phi, theta, psi, u, v, w, x, y, z])
            if length(res) >= 1
                data = typecast(res.Data(1:12), 'double');
                pose.translation = {data(10), data(11), data(12)};
                pose.rotation = {data(4), data(5), data(6)};
                src.UserData = pose;
                % [p, q, r, phi, theta, psi, u, v, w, x, y, z] = deal(data(1), data(2), data(3), data(4), data(5), data(6), data(7), data(8), data(9), data(10), data(11), data(12)) ;
            end
        end
    end

    % todo: load initial conditions file
    function obj = initialize(obj)
    end
        
    function control(obj, thrust, phi, theta, psi)
      % Define the data to be sent (4 values)
      % phi = min(max(phi, -deg2rad(20)), deg2rad(20));
      % theta = min(max(theta, -deg2rad(20)), deg2rad(20));
      % thrust = min(max(thrust, 0), 1);

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
