classdef LowPass < handle

  properties
    a
    y
  end

  methods

    % LowPass constructor
    % alpha: filter coefficient. If u is relatively noise free, set alpha close to unity. 
    % If u is noisy, set alpha close to zero.
    function obj = LowPass(smoothing)
      if smoothing < 0 || smoothing > 1
        error('cutoff must be greater than 0');
      end
      obj.a = smoothing;
      obj.y = 0;
    end

    function y = filter(obj, u)
    if obj.y == 0
        obj.y = u;
    end
      alpha = obj.a;
      y = (alpha * u) + (1 - alpha) * obj.y;
      obj.y = y;
    end
  end

end
