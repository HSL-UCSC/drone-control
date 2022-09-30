function pressure = parse_ble_pressure(data, scale)
    % Data - 1x2 matrix (pressure)
    % Data should really only have a value in first position unless negative
    
    % Pressure value
    if(data(2) < 10) % Positive angle 
        pressure = (256*data(2) + data(1))/scale;
    else
        pressure = -((256*(255 - data(2)) + (256-data(1))))/scale;
    end
    
end

