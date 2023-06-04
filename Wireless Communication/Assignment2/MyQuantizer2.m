function [quantizedSignal,varLin,varSat,SNqR] = MyQuantizer2(unquantizedSignal,Vp,N)
%% Quantize, equally spaced [-V_p, V_p], a signal     
%
% Inputs:
%       
%       unquantizedsignal:  The signal vector which is not quantized 
%       Vp:                 The boundary [-V_p, V_p]
%       N:                  Bits
% 
% Outputs:
% 
%       quantizedSignal:    Equally spaced, [-V_p, V_p], signal vector
%       varLin:             Estimated variance of the linear errors
%       varSat:             Estimated variance of the saturated errors
%       SNqR:               Estimated Signal to Quantization Noise Power Ratio in dB
% 
% Author: Farat Ghader Kourehpaz and Ghaith Ghadri
%
%% Script
% Sampled signal
s = unquantizedSignal';

% Number of bits
bits = N;

% Number of quantization levels
L = 2^bits;

% Quantization step size
delta = (2*Vp) / (L);

% Quantization levels
Llow = -Vp+delta/2;
Lhigh = Vp-delta/2;
quantizationLevels = Llow:delta:Lhigh;

% Take the difference between the row vector "level" and the column vector
% "s'" which produces a matrix
diff = quantizationLevels - s';

% Quantized signal
[~, idx] = min(abs(diff), [], 2);
quantizedSignal = quantizationLevels(idx);

%% Empirical estimation of the SNqR
% Calculate quantization noise
quantization_noise = s-quantizedSignal;

% Calculate power of signal and quantization noise
%  mean(s.^2)
signal_power = sum(s.^2)/length(s);
quantization_noise_power = sum(quantization_noise.^2)/length(quantization_noise);

% Calculate SNqR (dB)
SNqR = 10*log10(signal_power/quantization_noise_power);

%% Empirical estimation of the variance of the linear and saturation error
% Calculate the estimated variance of the linear and saturation 
E_lin = [];
E_sat = [];
for i = 1:length(unquantizedSignal)
    if (unquantizedSignal(i) < -Vp )
        E_lin = [E_lin, (-Vp - quantizedSignal(i))];
        E_sat = [E_sat, (-Vp-unquantizedSignal(i))];
    elseif (unquantizedSignal(i) > Vp)
        E_lin = [E_lin, (Vp - quantizedSignal(i))];
        E_sat = [E_sat, (Vp-unquantizedSignal(i))];
    else
        E_lin = [E_lin, (unquantizedSignal(i) - quantizedSignal(i))];
    end
end
varSat = sum(E_sat.^2)/(length(quantizedSignal));
varLin = sum(E_lin.^2)/(length(quantizedSignal));



