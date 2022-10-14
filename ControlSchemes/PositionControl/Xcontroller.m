function [x_acc_d, pid_output, X_pid_err] = Xcontroller(X_pid, x_d, x, x_dot, dt, tau, k)
    MAX_ACC = 900; % Max acceleration using thrust=value(0,255) assuming thrust>=thrustTrim(140)
        
    %% -------------------- FIRST PID BLOCK ------------------------------
    % Gains
    K_p = 150; % Shouldnt do anything with 100 (maybe 1 degree commanded) % 250,0,1 best for just commanding x
    K_i = 15; % 350,5,200 was last gains 300,10,100
    K_d = 50;
    
    % Proportional
    X_pid_err.x_curr_error = x_d - x;
    pid_p = K_p * X_pid_err.x_curr_error; 
    
    % Derivative
    X_pid_err.deriv = -K_d*x_dot; % taking off minus sign
    pid_d = X_pid_err.deriv;
    
    
    %Integral
    X_pid_err.x_cumm_error = X_pid.x_cumm_error + K_i*dt*(X_pid_err.x_curr_error);
    pid_i = X_pid_err.x_cumm_error;

        % Integral saturations
%     if(pid_i > MAX_OUT)
%         pid_i = MAX_OUT;
%     end
%     if(pid_i < -MAX_OUT)
%         pid_i = -MAX_OUT;
%     end
    
    % Store previous error
    X_pid_err.x_prev = X_pid_err.x_curr_error;
    
    % Final PID result
    pid_output = [pid_p, pid_i, pid_d];
    
    x_acc_d = pid_p + pid_i + pid_d;
    x_acc_d = min(max(-MAX_ACC, x_acc_d), MAX_ACC);
    
    
end