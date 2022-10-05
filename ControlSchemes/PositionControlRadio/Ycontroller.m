function [y_acc_d, pid_output, Y_pid_err] = Ycontroller(Y_pid, y_d, y, y_dot, dt, tau, k)
    MAX_ACC = 900; % Max acceleration using thrust=value(0,255) assuming thrust>=thrustTrim(140)
    
    %% -------------------- FIRST PID BLOCK ------------------------------
    % Gains
    K_p = 350; % Shouldnt do anything with 100 (maybe 1 degree commanded)
    K_i = 5;
    K_d = 200; %
    
    % Proportional
    Y_pid_err.y_curr_error = y_d - y;
    pid_p = K_p * Y_pid_err.y_curr_error; 
    
    % Derivative
    Y_pid_err.deriv = -K_d*y_dot;
    pid_d = Y_pid_err.deriv;
    
    
    %Integral
    Y_pid_err.y_cumm_error = Y_pid.y_cumm_error + K_i*dt*(Y_pid_err.y_curr_error);
    pid_i = Y_pid_err.y_cumm_error;
    
        % Integral saturations
%     if(pid_i > MAX_OUT)
%         pid_i = MAX_OUT;
%     end
%     if(pid_i < -MAX_OUT)
%         pid_i = -MAX_OUT;
%     end
    
    % Store previous error
    Y_pid_err.y_prev = Y_pid_err.y_curr_error;
    
    % Final PID result
    pid_output = [pid_p, pid_i, pid_d];
    
    y_acc_d = pid_p + pid_i + pid_d;
    y_acc_d = min(max(-MAX_ACC, y_acc_d), MAX_ACC);
    
end