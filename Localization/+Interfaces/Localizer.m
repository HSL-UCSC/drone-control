classdef (Abstract) Localizer < handle

    % Interface to a localization provider
    %  A localization provider is any source that can provide position data on a simulation entity.
    %  An example concrete implementation could be a motion capture camera system, a GPS, or simulated/filtered versions of either.

    methods (Abstract)
        % TODO: strong preference for t at the end
        pose = get_pose(obj, subject, segment, filtered)
        success = initialize(obj)
        success = shutdown(obj)
    end
end
