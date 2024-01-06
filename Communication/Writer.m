classdef (Abstract) Writer
    % Writer interface for implementing general communications interfaces

    properties
        inititialized bool
    end

    methods (Abstract)

        function packet = write(obj, packet)
        end

    end

end
