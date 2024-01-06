classdef (Abstract) Reader
    % Reader interface for implementing general communications interfaces

    properties
        inititialized bool
    end

    methods (Abstract)

        function [data, t] = read(obj, char)
        end

    end

end
