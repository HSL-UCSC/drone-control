classdef LowPass

  properties
    alpha
    y
  end

  methods

    % LowPass constructor
    % alpha: filter coefficient. If u is relatively noise free, set alpha close to unity. 
    % If u is noisy, set alpha close to zero.
    function obj = LowPass(alpha)
      if alpha < 0 || alpha > 1
        error('Alpha must be between 0 and 1');
      end
      obj.alpha = alpha;
      obj.y = 0;
    end

    function y = filter(obj, u)
      obj.y = (1 - obj.alpha) * obj.y + obj.alpha * u;
      y = obj.y;
    end
  end

end
