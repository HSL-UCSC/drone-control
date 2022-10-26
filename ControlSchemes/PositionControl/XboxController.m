classdef XboxController < handle
    properties (Access = private) 
        cntrlState = 0;
        cntrlStatePtr = 0;
        cntrlId = 0;
    end 
    methods
        function init = init(obj)
            % Verify Xinput library has not been loaded yet
            if libisloaded('XInput1_4')
                unloadlibrary XInput1_4
            end
            
            % Load Xinput dll and header library
            addpath(fullfile('c:','Windows','System32'))
            addpath(fullfile('c:','Program Files (x86)','Windows Kits','10','Include','10.0.19041.0','um'))
            [notfound,warnings] = loadlibrary('XInput1_4.dll','Xinput.h');
            
            % Construct the InputState C structure
            gamePadStruc.wButtons = 0;
            gamePadStruc.bLeftTrigger = 0;
            gamePadStruc.bRightTrigger = 0;
            gamePadStruc.sThumbLX = 0;
            gamePadStruc.sThumbLY = 0;
            gamePadStruc.sThumbRX = 0;
            gamePadStruc.sThumbRY = 0;
            gamepadState = libstruct('s_XINPUT_GAMEPAD',gamePadStruc);
            
            cntrStateStruc.dwPacketNumber = 0;
            cntrStateStruc.Gamepad = gamepadState;
            obj.cntrlState = libstruct('s_XINPUT_STATE',cntrStateStruc);
            
            % Create a pointer to the InputState C structure
            obj.cntrlStatePtr = libpointer('s_XINPUT_STATE', obj.cntrlState);

            % Determine ID of controller
            obj.cntrlId = 0;
            if(calllib('XInput1_4','XInputGetState',0,obj.cntrlStatePtr) == 0)
                obj.cntrlId = 0;
            elseif(calllib('XInput1_4','XInputGetState',1,obj.cntrlStatePtr) == 0)
                obj.cntrlId = 1;
            elseif(calllib('XInput1_4','XInputGetState',2,obj.cntrlStatePtr) == 0)
                obj.cntrlId = 2;
            elseif(calllib('XInput1_4','XInputGetState',3,obj.cntrlStatePtr) == 0)
                obj.cntrlId = 3;
            end
        end

        function [thrust,yaw,pitch,roll] = getState(obj)
            % Fetch latest controller state
            calllib('XInput1_4','XInputGetState',obj.cntrlId,obj.cntrlStatePtr);
            
            % Parse controller input into flight control command
            thrust = uint8(min(max(obj.cntrlState.Gamepad.sThumbLY / 128.5, 0), 255));
            yaw = uint8(min(max(((obj.cntrlState.Gamepad.sThumbLX + 32768) / 257), 0), 255));
            pitch = uint8(min(max(((obj.cntrlState.Gamepad.sThumbRY + 32768) / 257), 0), 255));
            roll = uint8(min(max(((obj.cntrlState.Gamepad.sThumbRX + 32768) / 257), 0), 255));
        end
        
    end
end