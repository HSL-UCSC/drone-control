function [T, output_z, Z_pid_err] = VZcont(Z_pid, z_d, z, v_z_d, dt, k)
    output1 = 0.0;
    output_z = 0.0;
    MAX_OUT = 10;
    MAX_T = 255;
    
    
    %% -------------------- FIRST PID BLOCK ------------------------------
    % Gains
    K_p_z1 = 4.0;
    K_i_z1 = 0.0;
    K_d_z1 = 0.0;
    
    Z_pid_err.z_curr_error = z_d - z;
    output1 = output1 + K_p_z1*Z_pid_err.z_curr_error; 
    
    Z_pid_err.z_cumm_error = Z_pid.z_cumm_error + Z_pid_err.z_curr_error*dt;
    output1 = output1 + K_i_z1 * Z_pid_err.z_cumm_error;
    
    %% -------------------- SECOND PID BLOCK ------------------------------
    % Gains
    K_p_z2 = 1.2*1.0;
    K_i_z2 = 0.5;
    K_d_z2 = 0.5;
    
    Z_pid_err.vz_curr_error = output1 - v_z_d; %v_z_d - output1;
    output_z = output_z + K_p_z2*Z_pid_err.vz_curr_error; 
    
    if k < 2
        Z_pid.vz_prev = Z_pid_err.vz_curr_error;
    end
    Z_pid_err.deriv = (Z_pid_err.vz_curr_error - Z_pid.vz_prev)/dt;
    [deriv_f, Z_pid_err.lpf_data] = lpf_2(Z_pid.lpf_data, Z_pid_err.deriv);
    Z_pid_err.deriv = deriv_f;
    output_z = output_z + K_d_z2 * Z_pid_err.deriv;
    
    Z_pid_err.vz_cumm_error = Z_pid.vz_cumm_error + Z_pid_err.vz_curr_error*dt;
    output_z = output_z + K_i_z2 * Z_pid_err.vz_cumm_error;
    
    Z_pid_err.vz_prev = Z_pid_err.vz_curr_error;
    
    % Need to convert to 0-255
    output_n = min(max(0.001, output_z), MAX_OUT);
    output_n = output_n*(MAX_T/MAX_OUT);
    %convert the value to 0 - 255
    T = uint8(output_n);
    
end