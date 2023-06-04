%% Quantizating a signal
% Sampled signal
unquantizedSignal = [-4 1 5];

% Number of bits
bits = 2;

% Minimum and maximum amplitudes
Vp = 4;

% Number of quantization levels
L = 2^bits;

% Quantization step size
delta = (2*Vp) / (L);

% Quantization levels
Llow = -Vp+delta/2;
Lhigh = Vp-delta/2;
quantizationLevels = Llow:delta:Lhigh;

% Quantized signal
[~, idx] = min(abs(quantizationLevels - unquantizedSignal'), [], 2);
quantizedSignal = quantizationLevels(idx);

%%
% Calculate quantization noise
quantization_noise = unquantizedSignal-quantizedSignal;

% Calculate power of signal and quantization noise
signal_power = sum(unquantizedSignal.^2)/length(unquantizedSignal);
quantization_noise_power = sum(quantization_noise.^2)/length(quantization_noise);

% Calculate SNqr
SNqR = 10*log10(signal_power/quantization_noise_power);
%%
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

%% Generate n-bit gray code
n = 2;
Vp = 4;
arr = [];
arr = [arr, "0"];
arr = [arr, "1"];

i = 2;
j = 0;

while true
    if (i >= 2^n)
        break;
    end
    for j = i:-1:1
        arr = [arr, arr(j)];
    end
    
    for j = 1:1:i
        arr(j) = "0"+arr(j);
    end

    for j = i+1:1:2*i
        arr(j) = "1"+arr(j);
    end

    i = i*2;
end
 
gray_arr = [];

if (n>1)
    % Number of quantization levels
    L = 2^n;
    
    % Quantization step size
    delta = (2*Vp) / (L);
    
    % Quantization levels
    Llow = -Vp+delta/2;
    Lhigh = Vp-delta/2;
    quantizationLevels = Llow:delta:Lhigh;
    gray_arr = [];
    
    
    for i = 1:length(quantizedSignal)
        for j = 1:length(quantizationLevels)
            if (quantizedSignal(i) == quantizationLevels(j))
                gray_arr = [gray_arr, arr(j)];
            end
        end
    end
    
    
    gray_arr = split(gray_arr, "", 2);
    gray_arr = gray_arr(gray_arr~="");
    gray_arr = str2double(gray_arr');
else
    gray_arr = arr;
end
%% Digital to analog converter (DAC)
n = 2;
Vp = 4;

% Number of quantization levels
L = 2^n;

% Quantization step size
delta = (2*Vp) / (L);

% Quantization levels
Llow = -Vp+delta/2;
Lhigh = Vp-delta/2;
quantizationLevels = Llow:delta:Lhigh;

% To do
% If n = 1, then gray code is singular [0, 1    ], if n = 2 then it is binary 

% Lets assume that 
bitStream = [0, 0, 1, 1, 1, 0];
% bitStream = [0, 0, 0, 0, 0, 1, 0, 1, 1, 0, 1, 0];

% Make it string
bitStream_str = string(bitStream);
% Save concatenated chars in an array 
gray_arr_reconst = [];

% Concatenate n chars at a time 
for i = 1:length(bitStream)
    if (mod(i, n) == 0)
        b = "";
        for j = n-1:-1:0
            b = b + bitStream_str(i-j); 
        end
        gray_arr_reconst = [gray_arr_reconst, b];
    else
        continue;
    end
end

estimatedSignal = [];
% Replace the gray code with its respective decimal value
for i = 1:length(gray_arr_reconst)
    for j = 1:length(arr)
        if (gray_arr_reconst(i) == arr(j))
            estimatedSignal(i) = quantizationLevels(j);
        else
            continue;
        end
    end
end
