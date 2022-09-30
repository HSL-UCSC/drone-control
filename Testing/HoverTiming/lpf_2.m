function [filter_data, lpData_new] = lpf_2(lpData, sample)
    %lpData_new = lpf_2_init(sample_freq, cut_freq);
    
    delay0 = sample - lpData.delay1*lpData.a1 - lpData.delay2*lpData.a2;
    % may need to check if delay0 becomes a NAN.
    if(~isfinite(delay0))
       delay0 = sample; 
    end
    filter_data = delay0*lpData.b0 + lpData.delay1*lpData.b1 + lpData.delay2*lpData.b2;
    
    lpData_new = lpData;
    
    lpData_new.delay2 = lpData.delay1;
    lpData_new.delay1 = delay0;
    
end