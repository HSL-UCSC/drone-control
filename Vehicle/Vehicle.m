classdef (Abstract) Vehicle

    properties
        x
        y
        z
        u
        v
        w
        phi
        theta
        psi
        p
        q
        r
        armed
        mode
    end

    methods (Abstract)
        
        obj = initialize(obj)     
        mode = control_mode(obj)
   
        % get vehicle state 12 vector
        [x, y, z, u, v, w, phi, theta, psi, p, q, r] = state(obj)
        control = control(obj, thrust, phi, theta, psi)
        armed = arm(obj)
        disarmed = disarm(obj)
    end
end

