function [out] = parse_ble(data, scale) % Scale is for decimal precision (choose what was chosen at the firmware level)
    if(data(2) < 10) % Positive angle 
        out = (256*data(2) + data(1))/scale;
    else
        out = -((256*(255 - data(2)) + (256-data(1))))/scale;
    end
end