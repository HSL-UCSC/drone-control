%% Clear past data
close all;  
clear all;
clc;

%% Metadata file creation

% Dont touch
ExperimentNum = 0;

% Select ID of drone (1,2,3,etc.)
DroneNum = 0;
% Select Expirament Type ("C", "O")
ExperimentType = "C";

filename = sprintf("AFOSR_Results/%s-%s_Metadata", int2str(DroneNum), ExperimentType);

save(filename)


