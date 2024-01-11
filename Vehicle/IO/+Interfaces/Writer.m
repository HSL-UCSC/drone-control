classdef (Abstract) Writer < handle
    % Writer interface for implementing general communications interfaces

    properties
        inititialized = false;
    end

    methods (Abstract)

        packet = write(obj, packet)

    end

end
