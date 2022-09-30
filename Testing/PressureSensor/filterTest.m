clear;
clc;
close all;


load("PressureDataSetQuickRiseAndFall.mat");

%% MOVING AVERAGE (125 sample delay (156ms delay)
windowSize = 25; 
b = (1/windowSize)*ones(1,windowSize);
a = 1;
y = filter(b,a,pressureValues);

plot(pressureValues);
hold on;
plot(y)



%% IIR Filter from Crazyflie
% ARGS: in - latest pressure value
%       attenuation - weighting factor
%       filt - storing previous pressure (filtered) values
% RETURN: out - final filtered pressure value

% pressureValues = pressureValues*100;
% filteredPressures = [];
% filttmp = 0
% IIR_SHIFT = 8;
% 
% %#define IMU_ACC_IIR_LPF_ATTENUATION (IMU_UPDATE_FREQ / (2 * 3.1415 * IMU_ACC_WANTED_LPF_CUTOFF_HZ))
% IMU_UPDATE_FREQ = 800; %Hz
% IMU_ACC_WANTED_LPF_CUTOFF_HZ = 4; %Hz
% IMU_ACC_IIR_LPF_ATTENUATION = (IMU_UPDATE_FREQ / (2 * 3.1415 * IMU_ACC_WANTED_LPF_CUTOFF_HZ));
% 
% %#define attenuation = IMU_ACC_IIR_LPF_ATT_FACTOR  (int)(((1<<IIR_SHIFT) / IMU_ACC_IIR_LPF_ATTENUATION) + 0.5)
% IMU_ACC_IIR_LPF_ATT_FACTOR = ((bitshift(1,IIR_SHIFT) / IMU_ACC_IIR_LPF_ATTENUATION) + 0.5);
% 
% attenuation = IMU_ACC_IIR_LPF_ATT_FACTOR;
% 
% for i=1:size(pressureValues,2)
%     if (attenuation > bitshift(1,IIR_SHIFT))%(1<<IIR_SHIFT))
%         attenuation = bitshift(1,IIR_SHIFT);%(1<<IIR_SHIFT);
%     else
%         if (attenuation < 1)
%             attenuation = 1;
%         end
%     end
%     
%     %   // Shift to keep accuracy
%     inScaled = bitshift(round(pressureValues(i)),IIR_SHIFT);
% %   // Calculate IIR filter
%     filttmp = filttmp + (bitshift(round(inScaled-filttmp), -IIR_SHIFT) * attenuation);
% %   // Scale and round
%   	out = bitshift(round(filttmp),-8) + bitshift(round(filttmp & bitshift(1,IIR_SHIFT-1)), -(IIR_SHIFT - 1));
% %   *filt = filttmp;
%     filteredPressures(i) = out;
%     i
% end

% int16_t iirLPFilterSingle(int32_t in, int32_t attenuation,  int32_t* filt)
% {
%   int32_t inScaled;
%   int32_t filttmp = *filt;
%   int16_t out;
% 
%   if (attenuation > (1<<IIR_SHIFT))
%   {
%     attenuation = (1<<IIR_SHIFT);
%   }
%   else if (attenuation < 1)
%   {
%     attenuation = 1;
%   }
% 
%   // Shift to keep accuracy
%   inScaled = in << IIR_SHIFT;
%   // Calculate IIR filter
%   filttmp = filttmp + (((inScaled-filttmp) >> IIR_SHIFT) * attenuation);
%   // Scale and round
%   out = (filttmp >> 8) + ((filttmp & (1 << (IIR_SHIFT - 1))) >> (IIR_SHIFT - 1));
%   *filt = filttmp;
% 
%   return out;
% }







%% ST DRONE IIR FILTER
% press_fil = [];
% pressure_previous = zeros(1,2);
% for i=1:size(pressureValues,2)
%     % Get current pressure value
%     curr_offsetted_pressure_reading = pressureValues(i);
%     
%     % IIR Filtering on pressure sensor
%     coeff_b0 = 0.01;
%     coeff_b1 = 0.01;
%     coeff_b2 = 0.001;
%     press_fil(i) = coeff_b0*pressureValues(i) + coeff_b1*pressure_previous(1) + coeff_b2*pressure_previous(2);
% 
%     % Shift IIR filter states
%     pressure_previous(2) = pressure_previous(1);
%     pressure_previous(1) = pressureValues(i);
% end
% 
% 
% figure()
% plot(pressureValues)
% title("Original");
% 
% figure()
% plot(press_fil)
% title("Filtered");




