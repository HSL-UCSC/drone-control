classdef VelocityController
  
  properties
    x_pid
    y_pid
    z_pid
    vx_pid
    vy_pid
    vz_pid
  end
  
  methods
    
    function obj = VelocityController(sample_frequency, cutoff_frequency, x_gains, y_gains, z_gains, vx_gains, vy_gains, vz_gains)
      
      arguments
        sample_frequency = 60;
        cutoff_frequency = 10;
        
        x_gains = Control.Gains(2, 0, 0);
        y_gains = Control.Gains(2, 0, 0);
        z_gains = Control.Gains(2, 0, 0);
        
        vx_gains = Control.Gains(4, 0, 0);
        vy_gains = Control.Gains(4, 0, 0);
        vz_gains = Control.Gains(5, 2, 0);
      end
      
      obj.x_pid = Control.PID(x_gains.kp, x_gains.ki, x_gains.kd, -1, 1, 0, sample_frequency, cutoff_frequency);
      obj.y_pid = Control.PID(y_gains.kp, y_gains.ki, y_gains.kd, -1, 1, 0, sample_frequency, cutoff_frequency);
      obj.z_pid = Control.PID(z_gains.kp, z_gains.ki, z_gains.kd, -.5, 1, 0, sample_frequency, cutoff_frequency);
      
      obj.vx_pid = Control.PID(vx_gains.kp, vx_gains.ki, vx_gains.kd, -20, 20, 0, sample_frequency, cutoff_frequency);
      obj.vy_pid = Control.PID(vy_gains.kp, vy_gains.ki, vy_gains.kd, -20, 20, 0, sample_frequency, cutoff_frequency);
      obj.vz_pid = Control.PID(vz_gains.kp, vz_gains.ki, vz_gains.kd, -20, 20, 0, sample_frequency, cutoff_frequency);
    end
    
    function [vx, vy, vz] = control(obj, target_state, current_state, dt)
      %
      %
      % float cosyaw = cosf(state->attitude.yaw * (float)M_PI / 180.0f);
      % float sinyaw = sinf(state->attitude.yaw * (float)M_PI / 180.0f);
      %
      % float setp_body_x = setpoint->position.x * cosyaw + setpoint->position.y * sinyaw;
      % float setp_body_y = -setpoint->position.x * sinyaw + setpoint->position.y * cosyaw;
      %
      % state_body_x = state->position.x * cosyaw + state->position.y * sinyaw;
      % state_body_y = -state->position.x * sinyaw + state->position.y * cosyaw;
      %
      % float globalvx = setpoint->velocity.x;
      % float globalvy = setpoint->velocity.y;
      
      u = current_state(7);
      v = current_state(8);
      w = current_state(9);
      
      cosyaw = cos(current_state(6));
      sinyaw = sin(current_state(6));
      
      body_xd = target_state(1) * cosyaw + target_state(2) * sinyaw;
      body_yd = -target_state(1) * sinyaw + target_state(2) * cosyaw;
      
      body_x = current_state(1) * cosyaw + current_state(2) * sinyaw;
      body_y = -current_state(1) * sinyaw + current_state(2) * cosyaw;
      
      [body_vxd, ~] = obj.x_pid.control(body_xd, body_x, dt);
      [body_vyd, ~] = obj.y_pid.control(body_yd, body_y, dt);
      [body_vzd, ~] = obj.z_pid.control(target_state(3), current_state(3), dt);
      
      body_vx = u * cosyaw + v * sinyaw;
      body_vy = -u * sinyaw + v * cosyaw;
      
      [vx, ~] = obj.vx_pid.control(body_vxd, body_vx, dt);
      [vy, ~] = obj.vy_pid.control(body_vyd, body_vy, dt);
      [vz, ~] = obj.vz_pid.control(body_vzd, w, dt);
    end
    
  end
  
end
