clear
clc
close all

fileID = fopen('log2.txt','r');
sizeA = [3 Inf];
A = fscanf(fileID,"%d  %d  %d",sizeA);
fclose(fileID);

m=1;
k=1;
R = [];
for i=1:size(A,2)
    R(m,:,k) = (A(:,i)/100)';
    m=m+1;
    if(mod(i,3)==0)
        k=k+1;
        m=1;
    end
end

%%
eul = rotm2eul(R, 'XYZ');

figure()
hold on;
plot(eul(:,1)*180/pi)
plot(eul(:,2)*180/pi)
plot(eul(:,3)*180/pi)