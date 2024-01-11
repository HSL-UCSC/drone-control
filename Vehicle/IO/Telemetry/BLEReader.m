classdef BLEReader < Interfaces.Reader & handle

    properties
        initialized = false;
        ble_connection;
    end

    methods

        function obj = BLEReader(ble_address)

            % BLE initialization
            obj.ble_connection = ble(ble_address);
            obj.ble_imu_char = characteristic(obj.ble_connection, "00000000-0001-11E1-9AB4-0002A5D5C51B", "00E00000-0001-11E1-AC36-0002A5D5C51B");
            obj.ble_arm_char = characteristic(obj.ble_connection, "00000000-0001-11E1-9AB4-0002A5D5C51B", "20000000-0001-11E1-AC36-0002A5D5C51B");
            obj.ble_bat_char = characteristic(obk.ble_connection, "00000000-0001-11E1-9AB4-0002A5D5C51B", "001D0000-0001-11E1-AC36-0002A5D5C51B");

            obj.characteristics = dictionary("imu", obj.ble_imu_char, "arm":obj.ble_arm_char, "bat":obj.ble_bat_char)
            % TODO: take a list of characteristics/callbacks?
            % BLE Subscriptions initialization
            % subscribe(obj.ble_imu_char)
            % obj.ble_imu_char.DataAvailableFcn = imu_callback;
            obj.initialized = true;
        end

        % TODO: read doesn't return timestamps, best I can tell
        function [data, timestamps] = read(obj, characteristic)

            if ~obj.inititialized
                return
            end

            ble_characteristic = obj.characteristics()
            [data, timestamps] = read(characteristic, 'latest');
        end

        function shutdown()
            % unsubscribe from BLE characterstics
        end

    end

end
