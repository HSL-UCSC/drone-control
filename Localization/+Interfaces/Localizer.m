classdef (Abstract) Localizer < handle

    % Interface to a localization provider
    %  A localization provider is any source that can provide position data on a simulation entity.
    %  An example concrete implementation could be a motion capture camera system, a GPS, or simulated/filtered versions of either.

    methods (Abstract)
        % TODO: strong preference for t at the end
        [t, x, y, z, phi, theta, psi] = get_position(obj, id, filtered)
        % [t, x_dot, y_dot, z_dot, phi_dot, theta_dot, psi_dot] = get_velocity(obj, id, filtered)
        % [t, x, y, z, phi, theta, psi, x_dot, y_dot, z_dot, phi_dot, theta_dot, psi_dot] = get_state(obj, id, filtered)
        shutdown()
    end

end
