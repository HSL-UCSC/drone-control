if not(isfolder('../yamlmatlab'))
    !git clone https://github.com/ewiger/yamlmatlab.git ../yamlmatlab/
end
if not(isfolder('../vision'))
    !git clone https://github.com/hsl-UCSC/vision.git ../vision/
end
if not(isfolder('../cybertx'))
    !git clone https://github.com/friend0/cybertx.git ../cybertx/
end

% todo: change to the correct branches/tags

% Add dependencies to the path
addpath('../vision/clients/matlab/MotionCapture');

% Save the path for future sessions
savepath;
