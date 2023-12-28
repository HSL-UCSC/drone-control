classdef (Abstract) Bidirectional
    % Bidirectional provides an interface to a general two-way communications protocol, supporting
    % reading and writing
    
    
    properties
        reader Communication.Reader
        writer Communication.Writer
    end
    
    methods (Abstract)
        
        init(obj)
        
        read(obj, characteristic)
        
        write(obj, packet)
        
    end
end

