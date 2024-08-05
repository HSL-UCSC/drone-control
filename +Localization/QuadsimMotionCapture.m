classdef QuadsimMotionCapture <  Localization.Interfaces.Localizer & handle
    
    properties
        vehicles;
    end
    
    methods
        
        
        
        function obj = QuadsimMotionCapture(vehicles)
            obj.vehicles = dictionary();
            for i = 1:length(vehicles)
                obj.vehicles(vehicles(i).id) = vehicles(i);
            end
        end
        
        function initialize(obj)
        end
        
        function success = shutdown(obj)
          obj.client.Uninitialize();
          success = true;
        end
        
        function register_vehicle(obj, vehicle)
          obj.vehicles(vehicle.id) = vehicle;
        end
        
        function pose = get_pose(obj, subject, segment, filtered)
          arguments
            obj
            subject
            segment = ""
            filtered = false
          end
          udp = obj.vehicles(subject).udprx;
          pose.translation = udp.UserData.translation;
          pose.rotation = udp.UserData.rotation;
          pose.velocity = udp.UserData.velocity;
        end
        
    end
    
end

