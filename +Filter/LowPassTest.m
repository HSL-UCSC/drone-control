% Example data: Sine wave with added noise
t = 0:0.01:10; % Time vector
x = sin(2*pi*0.5*t) + 0.5*randn(size(t)); % Noisy sine wave

% Parameters for EMA
alpha = 0.1; % Smoothing factor (0 < alpha <= 1)
ema = zeros(size(x)); % Preallocate EMA output
filt = Filter.LowPass(.1);
ema(1) = filt.filter(1); % Initialize EMA with the first data point


% Apply EMA filter
for k = 2:length(x)
    ema(k) = alpha * x(k) + (1 - alpha) * ema(k-1);
end

% Plot original data and filtered data
figure;
plot(t, x, 'b', 'DisplayName', 'Noisy Data');
hold on;
plot(t, ema, 'r', 'DisplayName', 'EMA Filtered Data');
legend;
xlabel('Time');
ylabel('Amplitude');
title('Exponential Moving Average Filter');
grid on;
