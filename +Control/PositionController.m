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
        
        x_gains = Control.Gains(1, 0, .05);
        y_gains = Control.Gains(1, 0, .05);
        z_gains = Control.Gains(5, 0, .15);
      end
      
      obj.x_pid = Control.PID(x_gains.kp, x_gains.ki, x_gains.kd, -12, 12, sample_frequency, cutoff_frequency);
      obj.y_pid = Control.PID(y_gains.kp, y_gains.ki, y_gains.kd, -12, 12, sample_frequency, cutoff_frequency);
      obj.z_pid = Control.PID(z_gains.kp, z_gains.ki, z_gains.kd, -5, 5, sample_frequency, cutoff_frequency);
    end
    
    function [ax, ay, az] = control(obj, target_state, current_state, dt)
      
      phi = current_state(4);
      theta = current_state(5);
      psi = current_state(6);
      ug = current_state(7);
      vg = current_state(8);
      wg = current_state(9);
      
      x_err = (target_state(1) - current_state(1));
      y_err = (target_state(2) - current_state(2));
      z_err = (target_state(3) - current_state(3));
      
      % these trig operations are the rotations from the inertial to body
      % frame, assuming projected onto the XY plane, i.e. assume roll and
      % pitch are zero
      x_vd = (x_err * cos(psi) + y_err * sin(psi));
      y_vd = (y_err * cos(psi) - x_err * sin(psi));
      
      ub = (ug * cos(psi) + vg * sin(psi));
      vb = (vg * cos(psi) - ug * sin(psi));
      
      [ax, ~] = obj.x_pid.control(x_vd, ub, dt);
      [ay, ~] = obj.y_pid.control(y_vd, vb, dt);
      [az, ~] = obj.z_pid.control(target_state(3), current_state(3), dt);
      az = az + .5;
    end
    
  end
  
end
