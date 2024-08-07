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
      
      phi = current_state(4);
      theta = current_state(5);
      psi = current_state(6);
      us = current_state(7);
      vd = current_state(8);
      wd = current_state(9);
      
      Rib = [cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
        cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
        -sin(theta), sin(phi)*cos(theta), cos(phi)*cos(theta)];
      
      x_err = (target_state(1) - current_state(1));
      y_err = (target_state(2) - current_state(2));
      z_err = (target_state(3) - current_state(3));
      
      x_vd = (x_err * cos(psi) + y_err * sin(psi));
      y_vd = (y_err * cos(psi) - x_err * sin(psi));
      
      us_body = (us * cos(psi) + vd * sin(psi));
      vs_body = (vs * cos(psi) - ud * sin(psi));
      
      [ax, ~] = obj.x_pid.control(x_vd, us_body, dt);
      [ay, ~] = obj.y_pid.control(y_vd, vs_body, dt);
      [az, ~] = obj.z_pid.control(target_state(3), current_state(3), dt);
    end
    
  end
  
end
