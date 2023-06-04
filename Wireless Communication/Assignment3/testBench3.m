%% Task 1
% Send zeros and calculate the power of noisySignal
% Noise power
noisePower = 0.01;

% The transmitted signal  
transmittedSignal = zeros(1, 100);

% Add noise to the transmitted signal
noisySignal = channel2(transmittedSignal, noisePower);

% Calculate the the power of noisySignal
N = length(noisySignal);
signal_power = sum(abs(noisySignal).^2)/N;

%% Task 2
% Create 1000 of the same symbols corresponding to [01] message for QPSK
% using average symbol power of 1
symbol = (-1/sqrt(2)) + 1i*(1/sqrt(2));
symbols = symbol.*ones(1, 1000);

% Variances
var = [0.01, 0.1, 1];

received_symbols = {};
for idx = 1:length(var)            
    % Add noise
    received_symbol = channel2(symbols, var(idx));                    

    % Save in cells
    received_symbols{idx} = received_symbol;
end

% Create plots
figure(1);
scatter(real(received_symbols{1}), imag(received_symbols{1}))
hold on;
% Add lines to separate quadrants
line([0 0], [-1 1], 'Color', 'k')
line([-1 1], [0 0], 'Color', 'k')

xlabel('Phase')
ylabel('Quadrature')
title('QPSK Constellation with Quadrant Separation and variance = 0.01')
grid on;
hold off;

figure(2);
scatter(real(received_symbols{2}), imag(received_symbols{2}))
hold on;
% Add lines to separate quadrants
line([0 0], [-2 2], 'Color', 'k')
line([-2 2], [0 0], 'Color', 'k')

xlabel('Phase')
ylabel('Quadrature')
title('QPSK Constellation with Quadrant Separation and variance = 0.1')
grid on;
hold off;

figure(3);
scatter(real(received_symbols{3}), imag(received_symbols{3}))
hold on;
% Add lines to separate quadrants
line([0 0], [-4 4], 'Color', 'k')
line([-4 4], [0 0], 'Color', 'k')

xlabel('Phase')
ylabel('Quadrature')
title('QPSK Constellation with Quadrant Separation and variance = 1')
grid on;
hold off;

%% Calculate SER and BER
BER_symbols = {};
SER_symbols = {};
for i = 1:3
    for j = 1:length(symbols)
        % Calculate angle in radians
        theta = angle(received_symbols{i}(j));
        % theta in [0,90] and [180, 270]
        if (theta*(180/pi) < 90 && theta > 0 || 360+theta*(180/pi)>180 && 360+theta*(180/pi)<270) 
            SER_symbols{i}(j) = 1;
            BER_symbols{i}(j) = 1;        
        end
        % theta in region [90, 180]
        if (theta*(180/pi)<180 && theta*(180/pi)>90)
            SER_symbols{i}(j) = 0;
            BER_symbols{i}(j) = 0;
        end
        % theta in region [270, 360]
        if (360+theta*(180/pi) > 270 && 360+theta*(180/pi) < 360)
            SER_symbols{i}(j) = 1;
            BER_symbols{i}(j) = 2;
        end
    end
end

SER = {};
BER = {};
for i = 1:3
    SER{i} = mean(SER_symbols{i});
    BER{i} = mean(BER_symbols{i});
end

%% Task 3
% Generate random binary bits
c = [0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0]';

% Map bits onto 16-QAM and BPSK constellation
s_16QAM = maponto16QAM(c);
s_BPSK = mapontoBPSK(c);

% SNR per symbol
SNR = linspace(2, 100, 15);

% SNRdB per symbol 
SNRdB = 20.*log10(SNR); 

% Assuming that the average power is 1 per symbol (No = A^2 / snr)
No = 1./SNRdB;

s_noisy_16QAM = channel2(s_16QAM, No(1));
s_noisy_BPSK = channel2(s_BPSK, No(1));

s_16QAM_detect = detect16QAM(s_noisy_16QAM);
s_BPSK_detect = detectBPSK(s_noisy_BPSK);
%% 
% Calculate the average energy
E_avg = mean(abs(s_16QAM).^2);
disp(E_avg)

%% Task 4
% SNR per symbol
SNRdB = 20*log10(logspace(20, 0, 15)); 

% Assuming that the average power is 1 per symbol (No = A^2 / SNR (dB) per symbol)
No = 1./(10.^(SNRdB./20));

% Bitstream
c_original = bitStream';

% Map bitstream to constellation symbols (16QAM)
s_16QAM = maponto16QAM(c_original);

% Map bitstream to constellation symbols (BPSK)
s_BPSK = mapontoBPSK(c_original);

c_16QAM_detect_cell = {};
c_BPSK_detect_cell = {};
for i = 1:length(SNRdB)
    % Add noise to the symbols (16QAM)
    s_noisy_16QAM = channel2(s_16QAM, No(i));

    % Add noise to the symbols (BPSK)
    s_noisy_BPSK = channel2(s_BPSK, No(i));   

    % Detect the bitrate
    s_16QAM_detected = detect16QAM(s_noisy_16QAM);
    
    % Detect the bitrate
    s_BPSK_detected = detectBPSK(s_noisy_BPSK);

    % Save bitstream in cells for respective variance (16QAM)
    c_16QAM_detect_cell{i} = s_16QAM_detected;

    % Save bitstream in cells for respective variance (BPSK)
    c_BPSK_detect_cell{i} = s_BPSK_detected;
end

%%

%Plot the emprirical values of the detected 16QAM and BPSK
BER_16QAM_exp = [];
BER_BPSK_exp = [];
for i = 1:length(SNRdB)
    BER_16QAM_exp = [BER_16QAM_exp, sum(c_16QAM_detect_cell{i} ~= c_original)/length(c_original)];
    BER_BPSK_exp = [BER_BPSK_exp, sum(c_BPSK_detect_cell{i} ~= c_original)/length(c_original)];
end

% Calculate the theoretical BER values of BPSK and 16QAM
BER_16QAM_th = [];
BER_BPSK_th = [];
for i = 1:length(SNRdB)
    BER_16QAM_th = [BER_16QAM_th , qfunc(sqrt((6*SNRdB(i))/15))];
    BER_BPSK_th = [BER_BPSK_th , 2*qfunc(sqrt(SNRdB(i)))];
end

% Plot BER curves for 16QAM and BPSK
figure;
semilogy(SNRdB, BER_16QAM_exp, 'o-' ,'LineWidth', 2)
hold on;
semilogy(SNRdB, BER_BPSK_exp, 'o-', 'LineWidth', 2);
semilogy(SNRdB, BER_16QAM_th, 'o-', 'LineWidth', 2);
semilogy(SNRdB, BER_BPSK_th, 'o-', 'LineWidth', 2);
grid on;
xlabel('SNR per symbol [dB]');
ylabel('BER');
title('BER for 16QAM and BPSK');
legend('16QAM experimental', 'BPSK experimental', '16QAM theoretical', 'BPSK theoretical');
