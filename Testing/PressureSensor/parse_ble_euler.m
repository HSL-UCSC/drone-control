function [thx, thy, thz] = parse_ble_euler(data, scale)
    % Data - 1x6 matrix (thx,thy,thz=0)
    
    % Thx - pitch
    if(data(2) < 10) % Positive angle 
        thx = (256*data(2) + data(1))/scale;
    else
        thx = -((256*(255 - data(2)) + (256-data(1))))/scale;
    end
    
    % Thy - roll
    if(data(4) < 10) % Positive angle
        thy = (256*data(4) + data(3))/scale;
    else
        thy = -((256*(255 - data(4)) + (256-data(3))))/scale;
    end
    
    % Thz - yaw
    if(data(6) < 10) % Positive angle
        thz = (256*data(6) + data(5))/scale;
    else
        thz = -((256*(255 - data(6)) + (256-data(5))))/scale;
    end
    
end