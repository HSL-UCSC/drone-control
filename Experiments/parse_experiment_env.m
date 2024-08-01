function vehicles = parse_experiment_env(path)

% parse env file
data = py.importlib.import_module('yaml');
file = py.open('/home/z0/Developer/drone-control/Experiments/run_quads.yaml', 'r');
py_dict = data.safe_load(file.read());
file.close();
vehicles = dictionary;
% Convert each key-value pair to the MATLAB map
for i = 1:length(py_dict{'vehicles'})
  v = dictionary(py_dict{'vehicles'}{i});
  vehicle_config = values(v);
  vehicle_name = string(keys(v))
  if vehicle_config{'model'} == 'Quadsim'
    vehicles{vehicle_name} = Quadsim.from_dict(struct(vehicle_config));
  end
  % value = values{i};
  
  % Handle the conversion if necessary, e.g., if values are not directly assignable
  % matlab_map(key) = double(value); % Example conversion to double
end
end
