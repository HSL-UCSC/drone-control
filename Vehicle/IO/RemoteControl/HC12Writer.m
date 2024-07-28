classdef HC12Writer < Interfaces.Writer & handle

    properties
        serial_device
    end

    methods

        function obj = HC12Writer(com_port, baud_rate)
            % HC12 initialization
            obj.serial_device = serialport(com_port, baud_rate);
            flush(obj.serial_device)
        end

        function packet = write(obj, packet)
            write(obj.serial_device, packet, "uint8")
        end

    end

end
