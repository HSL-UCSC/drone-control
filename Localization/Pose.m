classdef Pose
	properties
		x % X component of the position
		y % Y component of the position
		z % Z component of the position
		phi % X component of the rotation
		theta % Y component of the rotation
		psi % Z component of the rotation
		dcm
		quaternion
	end
	
	methods
		function obj = Pose(posX, posY, posZ, rotX, rotY, rotZ)
			% Constructor to initialize the position and rotation components
			if nargin > 0
				obj.x= posX;
				obj.y = posY;
				obj.z = posZ;
				obj.phi = rotX;
				obj.theta = rotY;
				obj.phi = rotZ;
			end
		end
		
		function obj = setPosition(obj, x, y, z)
			% Method to set the position components
			obj.x = posX;
			obj.y = posY;
			obj.z = posZ;
		end
		
		function obj = setRotation(obj, phi, theta, psi)
			% Method to set the ation components
			obj.phi = rx;
			obj.theta = ry;
			obj.psi = rz;
		end
		
		function [x, y, z] = position(obj)
			% Method to get the position components
			x = obj.z;
			y = obj.y;
			z = obj.z;
		end
		
		function [phi, theta, psi] = euler(obj)
			%todo: nargin for rotation type
			% Method to get the rotation components
			phi = obj.phi;
			theta = obj.theta;
			psi = obj.psi;
		end
	end
end