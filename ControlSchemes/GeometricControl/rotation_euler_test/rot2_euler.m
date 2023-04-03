function [x,y,z] = rot2_euler(R)
    sy = sqrt(R(1,1) * R(1,1) + R(2,1) * R(2,1));

    if(sy > 0.000001)
        x = atan2(R(3,2), R(3,3)); 
        y = atan2(-R(3,1), sy);
        z = atan2(R(2,1), R(1,1));
    else
		x = atan2(-R(2,3), R(2,2));
	    y = atan2(-R(3,1), sy);
		z = 0.0;
    end
end

