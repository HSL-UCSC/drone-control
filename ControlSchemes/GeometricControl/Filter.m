classdef Filter 
    methods     ( Static = true )
        function lpfData = lpf_2_init(sample_fr, cutt_fr, start_sample)
            fr = sample_fr/cutt_fr;
            ohm = tan(pi/fr);
            c = 1 + 2*cos(pi/4)*ohm + ohm*ohm;
            
            lpfData.b0 = ohm*ohm/c;
            lpfData.b1 = 2*lpfData.b0;
            lpfData.b2 = lpfData.b0;
            
            lpfData.a1 = 2*(ohm*ohm - 1)/c;
            lpfData.a2 = (1 - 2*cos(pi/4)*ohm + ohm*ohm)/c;
            
            lpfData.delay1 = start_sample;
            lpfData.delay2 = start_sample;
        end

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
        
    end
end