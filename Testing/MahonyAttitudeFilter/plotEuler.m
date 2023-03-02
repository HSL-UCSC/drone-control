clear
clc
close all


load Rhat.mat


eul = [];
for i=1:size(Rhat,1)
    eul(i,:) = rotm2eul(Rhat(i,:,:)); 
end

% Roll
plot(eul(:,1))