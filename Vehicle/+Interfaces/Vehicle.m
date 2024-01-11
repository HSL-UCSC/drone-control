classdef (Abstract) Vehicle

    methods (Abstract)
        state()
        control(obj, input)
    end
    
end

