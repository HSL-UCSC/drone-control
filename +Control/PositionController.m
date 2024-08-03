classdef PositionController
  
  properties
    x_pid
    y_pid
    z_pid
  end
  
  methods
    
    function obj = PositionController(sample_frequency, cutoff_frequency, x_gains, y_gains, z_gains)
      
      arguments
        sample_frequency = 60;
        cutoff_frequency = 10;
        
        x_gains = Control.Gains(.300, 0, .1);
        y_gains = Control.Gains(.300, 0, .1);
        z_gains = Control.Gains(1, 0, 0);
      end
      
      obj.x_pid = Control.PID(x_gains.kp, x_gains.ki, x_gains.kd, -12, 12, sample_frequency, cutoff_frequency);
      obj.y_pid = Control.PID(y_gains.kp, y_gains.ki, y_gains.kd, -12, 12, sample_frequency, cutoff_frequency);
      obj.z_pid = Control.PID(z_gains.kp, z_gains.ki, z_gains.kd, -1, 1, sample_frequency, cutoff_frequency);
    end
    
    function [ax, ay, az] = control(obj, target_state, current_state, dt)
      [ax, ~] = obj.x_pid.control(target_state(1), current_state(1), dt);
      [ay, ~] = obj.y_pid.control(target_state(2), current_state(2), dt);
      [az, ~] = obj.z_pid.control(target_state(3), current_state(3), dt);
    end
    
  end
  
end
