classdef (Abstract) Reader < handle
    % Reader interface for implementing general communications interfaces

    properties
        inititialized = false;
    end

    methods (Abstract)

        [data, t] = read(obj, char)

    end

end
