classdef LowPassFilter < Interfaces.Filter

    properties
        coefficients
        delay
    end
    methods

        function obj = LowPassFilter(sample_frequency, cut_frequency, start_sample)
            fr = sample_frequency / cut_frequency;
            ohm = tan(pi / fr);
            c = 1 + 2 * cos(pi / 4) * ohm + ohm * ohm;

            obj.coefficients.b0 = ohm * ohm / c;
            obj.coefficients.b1 = 2 * obj.coefficients.b0;
            obj.coefficients.b2 = obj.coefficients.b0;

            obj.coefficients.a1 = 2 * (ohm * ohm - 1) / c;
            obj.coefficients.a2 = (1 - 2 * cos(pi / 4) * ohm + ohm * ohm) / c;

            obj.delay.delay1 = start_sample;
            obj.delay.delay2 = start_sample;
        end

        function warmup(obj)
        end

        function filtered_sample = filter(obj, sample)
            delay0 = sample - (obj.coefficients.a1 * obj.delay.delay1) - (obj.coefficients.a2 * obj.delay.delay2);
            % may need to check if delay0 becomes a NAN.
            if (~isfinite(delay0))
                delay0 = sample;
            end

            filtered_sample = obj.coefficients.b0 * delay0 + obj.coefficients.b1 * obj.coefficients.delay1 + obj.coefficients.b2 * obj.coefficients.delay2;

            obj.delay.delay1 = delay0;
            obj.delay.delay2 = obj.delay.delay1;

        end

    end

end
