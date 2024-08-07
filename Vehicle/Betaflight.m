classdef Betaflight < Interfaces.Multirotor
  
  properties
    mass_kg = .060;
    control_mode = "manual";
    max_angle = 12;
    min_thrust = 0;
    max_thrust = 1;
    initialized = false;
    armed = false;
    control_writer;
    id
  end
  
  methods
    
    function obj = Betaflight(com_port, baud_rate, id, line, channels)
      
      arguments
        com_port
        baud_rate = 115200;
        id = "";
        line = 1;
        channels = 4;
      end
      
      % TODO: handle case of non-nil reader/writer to support dependency injection
      obj.id = id;
      obj.control_writer = CyberTXWriter(com_port, baud_rate, line, channels);
      
    end
    
    function arm(obj)
    end
    
    function disarm(obj)
    end
    
    function control(obj, comm_thr_d, comm_phi_d, comm_theta_d, comm_yaw_d)
      % map inputs to PPM range of 1000-2000
      % Thrust: 1000-2000 (1000 min, 1500 mid, 2000 max)
      % Roll: 1000-2000 (1000 left, 1500 mid, 2000 right)
      % Pitch: 1000-2000 (1000 forward, 1500 mid, 2000 reverse)
      % Yaw: 1000-2000 (1000 left, 1500 mid, 2000 right)
      attitude_scaling = 250 / obj.max_angle;
      thr_scaling = 500;
      comm_thr_d = max(1500, min(2000, 1000 + thr_scaling * comm_thr_d));
      comm_phi_d = max(1000, min(2000, 1500 + attitude_scaling * comm_phi_d));
      comm_theta_d = max(1000, min(2000, 1500 + attitude_scaling * comm_theta_d));
      comm_yaw_d = max(1000, min(2000, 1500 + attitude_scaling * comm_yaw_d));
      
      % attitude command packet follows AETR
      attitude_command_packet = obj.control_writer.Command(comm_phi_d, comm_theta_d, comm_thr_d, comm_yaw_d);
      obj.control_writer.write(attitude_command_packet);
    end
    
  end
  
end
