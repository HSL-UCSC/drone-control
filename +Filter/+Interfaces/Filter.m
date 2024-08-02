classdef (Abstract) Filter < handle
  
  methods (Abstract)
    warmup()
    filter()
  end
  
end
