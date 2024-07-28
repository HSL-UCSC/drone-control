s = SimVehicle("localhost", 25001)
[x, y, z, u, v, w, phi, theta, psi, ~] = s.get_position()
clear s