classdef XboxController < handle
    properties (Access = private) 
        cntrlState = 0;
        cntrlStatePtr = 0;
        cntrlId = 0;

        XINPUT_GAMEPAD_B = 0x2000;                  % Manual override switch
        XINPUT_GAMEPAD_LEFT_SHOULDER = 0x0100;      % Calibration switch
        XINPUT_GAMEPAD_RIGHT_SHOULDER = 0x0200;     % Arming switch
        XINPUT_GAMEPAD_START = 0x0010;              % Start landing sequence
        XINPUT_GAMEPAD_BACK = 0x0020;               % Toggle setpoint mode (controller, waypoint file)
        XINPUT_GAMEPAD_DPAD_LEFT = 0x0004;          % Previous setpoint
        XINPUT_GAMEPAD_DPAD_RIGHT = 0x0008;         % Next setpoint
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

        function [thrust,yaw,pitch,roll,calibrate,arm,override,land,setpointMode,setpointPrev,setpointNext] = getState(obj)
            % Fetch latest controller state
            calllib('XInput1_4','XInputGetState',obj.cntrlId,obj.cntrlStatePtr);
            
            % Parse controller input into flight control command
            thrust = min(max(obj.cntrlState.Gamepad.sThumbLY / 128.5, 0), 255);
            yaw = min(max(((obj.cntrlState.Gamepad.sThumbLX + 32768)*60 / 65536 - 30), -30), 30);
            pitch = -min(max(((obj.cntrlState.Gamepad.sThumbRY + 32768)*60 / 65536 - 30),-30), 30);
            roll = min(max(((obj.cntrlState.Gamepad.sThumbRX + 32768)*60 / 65536 - 30), -30), 30);

            % Manual override switch
            override = 0;
            if(bitand(obj.cntrlState.Gamepad.wButtons, obj.XINPUT_GAMEPAD_B))
                override = 1;
            end

            % Arming switch
            arm = 0;
            if(bitand(obj.cntrlState.Gamepad.wButtons, obj.XINPUT_GAMEPAD_RIGHT_SHOULDER))
                arm = 1;
            end

            % Calibration
            calibrate = 0;
            if(bitand(obj.cntrlState.Gamepad.wButtons, obj.XINPUT_GAMEPAD_LEFT_SHOULDER))
                calibrate = 1;
            end

            % Initiate landing
            land = 0;
            if(bitand(obj.cntrlState.Gamepad.wButtons, obj.XINPUT_GAMEPAD_START))
                land = 1;
            end

            % Setpoint mode (waypoint file, xbox control)
            setpointMode = 0;
            if(bitand(obj.cntrlState.Gamepad.wButtons, obj.XINPUT_GAMEPAD_BACK))
                setpointMode = 1;
            end

            % Setpoint previous
            setpointPrev = 0;
            if(bitand(obj.cntrlState.Gamepad.wButtons, obj.XINPUT_GAMEPAD_DPAD_LEFT))
                setpointPrev = 1;
            end

            % Setpoint next
            setpointNext = 0;
            if(bitand(obj.cntrlState.Gamepad.wButtons, obj.XINPUT_GAMEPAD_DPAD_RIGHT))
                setpointNext = 1;
            end

        end
        
    end
end