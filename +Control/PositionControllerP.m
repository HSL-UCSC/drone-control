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
        
        x_gains = Control.Gains(.2, 0, .1);
        y_gains = Control.Gains(.2, 0, .1);
        z_gains = Control.Gains(2, 0, -1);
      end
      
      obj.x_pid = Control.PID(x_gains.kp, x_gains.ki, x_gains.kd, -12, 12, sample_frequency, cutoff_frequency);
      obj.y_pid = Control.PID(y_gains.kp, y_gains.ki, y_gains.kd, -12, 12, sample_frequency, cutoff_frequency);
      obj.z_pid = Control.PID(z_gains.kp, z_gains.ki, z_gains.kd, -1, 1, sample_frequency, cutoff_frequency);
    end
    
    function [ax, ay, az] = control(obj, target_state, current_state, dt)
      
      psi = current_state(6);
      u = current_state(7);
      v = current_state(8);
      w = current_state(9);
      
      x_err = (target_state(1) - current_state(1));
      y_err = (target_state(2) - current_state(2));
      
      xs_body = (current_state(1) * cos(psi) + current_state(2) * sin(psi));
      ys_body = (current_state(2) * cos(psi) - current_state(1) * sin(psi));
      
      xd_body = (target_state(1) * cos(psi) + target_state(2) * sin(psi));
      yd_body = (target_state(2) * cos(psi) - target_state(1) * sin(psi));
      
      % x_err = (target_state(1) - current_state(1));
      % y_err = (target_state(2) - current_state(2));
      %
      % xd_b = (target_state(1) * cos(psi) + target_state(2) * sin(psi));
      % yd_b = (target_state(2) * cos(psi) - target_state(1) * sin(psi));
      %
      % x_b = (current_state(1) * cos(psi) + current_state(2) * sin(psi));
      % y_b = (current_state(2) * cos(psi) - current_state(1) * sin(psi));
      
      [vx, ~] = obj.x_pid.control(xd_body, xs_body, dt);
      [vy, ~] = obj.y_pid.control(yd_body, ys_body, dt);
      [vz, ~] = obj.z_pid.control(target_state(3), current_state(3), dt);
    end
    
  end
  
end
