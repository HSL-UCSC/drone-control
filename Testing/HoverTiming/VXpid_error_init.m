function X_pid = VXpid_error_init(reset_pid, OUT_FREQ, CUT_OFF_FREQ_POS)

    persistent x_curr_error
    persistent vx_curr_error
    persistent vx_cumm_error
    persistent vx_prev
    persistent deriv
    
    lpf_data = lpf_2_init(OUT_FREQ, CUT_OFF_FREQ_POS, 0.0);
    
    if reset_pid == 1
        x_curr_error = 0.0;
        vx_curr_error = 0.0;
        vx_cumm_error = 0.0;
        vx_prev = 0.0;
        deriv = 0.0;
    end
    
    X_pid.x_curr_error = x_curr_error;
    X_pid.vx_curr_error = vx_curr_error;
    X_pid.vx_cumm_error = vx_cumm_error;
    X_pid.vx_prev = vx_prev;
    X_pid.deriv = deriv;
    X_pid.lpf_data = lpf_data;
end

