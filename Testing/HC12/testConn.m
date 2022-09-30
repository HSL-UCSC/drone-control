clear;

% device = serialport("COM3",9600);
string1 = [65 66 67 68];
obj1 = instrfind('Type', 'serial', 'Port', 'COM3', 'Tag', '');
set(obj1, 'timeout',.5);
if isempty(obj1)
    obj1 = serial('COM3');
else
    disp("here")
    fclose(obj1);
    obj1 = obj1(1)
end
fopen(obj1);

wTimes = [];

for i=1:1000
    tic
    fwrite(obj1,string1,'uint8')
    t = toc;
    wTimes(i) = t;
end

% A=fread(obj1,2)
fclose(obj1);
% for i=1:1
% %     write(device,2,"uint8")
%     pause(1)
% %     read(device,5,"uint8")
%     A = fread(obj1,1)
% end