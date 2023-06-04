% Create bitstream 
bit = randi([0, 1], [1, 100000])';

% encode the received bitstream
r74 = encodeHAM74(bit);
r1511 = encodeHAM1511(bit);

% Modulate the signal using BPSK
r_mod_74 = mapontoBPSK(r74);
r_mod_1511 = mapontoBPSK(r1511);

% Create SNR per bit with the range [-10, 15]
SNRdB = 20*log10(logspace(-0.5, 0.75, 15));

% Solve N0
N0 = 1./SNRdB;

% Add noise for all 15 variances
b74_BPSK_detect = {};
b1511_BPSK_detect = {};
for i = 1:length(SNRdB)

    % Add noise to the symbols (BPSK)
    s74_noisy_BPSK = channel2(r_mod_74, N0(i));
    S1511_noisy_BPSK = channel2(r_mod_1511, N0(i));

    % Detect the bitrate
    s74_BPSK_detect = detectBPSK(s74_noisy_BPSK);
    s1511_BPSK_detect = detectBPSK(S1511_noisy_BPSK);

    % Decode the bitrate
    bEst74=decode74(s74_BPSK_detect);
    bEst1511=decode1511(s1511_BPSK_detect);

    % Save bitstream in cells for respective variance (BPSK)
    b74_BPSK_detect{i} = bEst74;
    b1511_BPSK_detect{i} = bEst1511;
end

% Compute the bit error rate (BER)
BER74_BPSK = [];
BER1511_BPSK = [];
for i = 1:length(SNRdB)
     BER74_BPSK = [BER74_BPSK, sum(b74_BPSK_detect{i} ~= bit)/length(bit)];

     BER1511_BPSK = [BER1511_BPSK, sum(b1511_BPSK_detect{i} ~= bit)/length(bit)];
end

%%
% Plot BER curves for 16QAM and BPSK
figure;
% semilogy(SNRdB, BER_16QAM_exp, 'o-' ,'LineWidth', 2)
% hold on;
semilogy(SNRdB, BER_BPSK, 'o-', 'LineWidth', 2);
% semilogy(SNRdB, BER_16QAM_th, 'o-', 'LineWidth', 2);
% semilogy(SNRdB, BER_BPSK_th, 'o-', 'LineWidth', 2);
grid on;
xlabel('SNR per symbol [dB]');
ylabel('BER');
title('BER for 16QAM and BPSK');
legend('16QAM experimental', 'BPSK experimental', '16QAM theoretical', 'BPSK theoretical');
