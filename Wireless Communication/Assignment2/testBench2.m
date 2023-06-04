% Given parameters
Es = 2;
M = 8;
bitstream = [0,1,1,1,1,0,1,1,0,1,0,0,1,0];

% Number of bits
k = log2(M);

% Find the equidistant distance d between each symbol 
i = 1:1:M;
d = sqrt((Es*M)/sum((2*i-1-M).^2));

% The amplitude levels
Ai = (2*i-1-M)*d;

% Number of the bitstream divided by amount of bits
N = floor(length(bitstream)/k);

if mod(length(bitstream), k) ~= 0
    % Zero-pad the bitstream to ensure that it contains a whole number of messages
    bitstream = [bitstream, zeros(1, k - mod(length(bitstream), k))];
    N = N + 1;
end

% Reshape the bitstream to a matrix
bitstream_reshape = reshape(bitstream', k, N)';

%% Create gray-code
% Create the first entry of "0" and "1"
arr = [];
arr = [arr, "0"];
arr = [arr, "1"];

% Every iteration of this loop generates 2*i codes from previously generated i codes.
i = 2;
j = 0;

while true
    if (i >= 2^k)
        break;
    end

    % append the previous arr in reverse order 
    for j = i:-1:1
        arr = [arr, arr(j)];
    end
    
    % append 0 to the first half
    for j = 1:1:i
        arr(j) = "0"+arr(j);
    end
    
    % append 1 to the second half
    for j = i+1:1:2*i
        arr(j) = "1"+arr(j);
    end
    
    % double i
    i = i*2;
end
%%
transmitSignal = zeros(size(bitstream_reshape, 1),1);
for i=1:size(bitstream_reshape, 1)
    row = num2str(bitstream_reshape(i, :));
    row(row == ' ') = ''; % remove spaces
    idx = find(strcmp(arr, row));
    transmitSignal(i) = Ai(idx);
end
transmitSignal = transmitSignal';
%% Plot rectangular pulses out of the transmitSignal
% Symbol time of 100 samples
T = 100;
% plot(t, transmitSignal)
plot(rectpulse(transmitSignal, T))
xlabel("time")
ylabel("Voltage levels [V]")
title("M-ary PAM transmit signal")

%% Demodulation of MPAM

receivedSignal = rectpulse(transmitSignal, 100);

% Sample the received signal T = 100
T = 100;
samples = T:T:length(receivedSignal);
sampledSignal = receivedSignal(samples);

% Find the equidistant distance d between each symbol 
i = 1:1:M;
d = sqrt((Es*M)/sum((2*i-1-M).^2));

% The amplitude levels
Ai = (2*i-1-M)*d;

% Define the decision regions Z_1, Z_2, ..., Z_M
Z = [];
for i = 1:1:length(sampledSignal)
    for j = 1:1:length(Ai)
        if (sampledSignal(i) < Ai(1)-d)
            Z(i) = Ai(1);
        end
        if (sampledSignal(i)>=Ai(j)-d && sampledSignal(i)< Ai(j)+d) 
            Z(i) = Ai(j);
        end
        if (sampledSignal(i) > Ai(end)+d)
            Z(i) = Ai(end);
        end
    end
end


% Gray code 
gray_arr = [];
for i = 1:1:length(Z)
    for j = 1:1:length(Ai)
        if (Z(i)==Ai(j))
            gray_arr = [gray_arr, arr(j)];
        end
    end
end

% Split string
gray_arr = split(gray_arr, "", 2);
% Remove empty cells
gray_arr = gray_arr(gray_arr~="")';
% Change string to double in array
estimatedBitStream = str2double(gray_arr);

errors = sum(estimatedBitStream ~= bitstream);
BER = errors/length(bitstream);

%%
% Given parameters
Es = 2;
M = 8;
bitstream = [0,1,1,1,1,0,1,1,0,1,0,0,1,0];

% Generate analog signal from bitstream
mod_signal = MyMPAM(bitstream, M, Es);

% Noise variance in dB
N0_dB = logspace(-2,0,20);

% Save BERs in an array
BER_arr = [];

for i = 1:length(N0_dB)
    % Put white noise on the signal
    noise = N0_dB(i)*randn(1, length(mod_signal));

    % Add noise to the analog signal
    mod_signal_err = mod_signal+noise;
    
    % Multiply the analog signal values with a rectangular function with
    % sample time T = 100
    mod_signal_err_rect = rectpulse(mod_signal_err, 100);

    % Demodulate the rectangular signal and calculate BER
    [estimatedBitstream, BER] = DemodulateMPAM(mod_signal_err_rect,M,Es,bitstream);
    BER_arr = [BER_arr, BER];
end
% % Demodulate received signal
% [estimatedBitstream, BER] = DemodulateMPAM(rectpulse(rx_signal, 100), M, Es, bitstream);

%%  
% Average energy
Es = 2;
% Original bitstream
% bitstream = [0,1,1,1,1,0,1,1,0,1,0,0,1,0];
% Modulation order
M = [2, 8];    
% Noise variance in dB
N0_dB = logspace(-6,0.6,50); 

BER = {};
% Modulate bits using PAM
for m = 1:length(M)
    tx_signal = MyMPAM(bitStream, M(m), Es);
    % Simulate channel noise for different noise variances
    for i = 1:length(N0_dB)
        rx_signal = tx_signal + N0_dB(i)*randn(size(tx_signal));
        % Demodulate received signal
        [estimatedBitstream, BER1] = DemodulateMPAM(rectpulse(rx_signal, 100), M(m), Es, bitStream);
        BER{m}(i) = BER1;
    end
end

% Plot BER vs. noise variance
figure;
semilogy(N0_dB, BER{1}, 'o-', 'LineWidth', 2);
hold on;
semilogy(N0_dB, BER{2}, 's-', 'LineWidth', 2);
grid on;
xlabel('Noise Variance (dB)');
ylabel('Bit Error Rate (BER)');
title('BER vs. Noise Variance');
legend('2PAM', '8PAM');


%%
bitstream = [0,1,1,1,1,0,1,1,0,1,0,0,1,0];
Es = 2;
M = 6;
T = 100;
N0_dB = logspace(-2,0,20); 

tx_signal = MyMPAM(bitstream, M, Es);
rx_signal = tx_signal + N0_dB(1)*randn(size(tx_signal));
rect_signal = rectpulse(rx_signal, T);

sampledSignal = rect_signal(1:T:end);
rx_signal = tx_signal + N0_dB(1)*randn(size(tx_signal));
[a] = DemodulateMPAM(rect_signal, M, Es, bitstream);


