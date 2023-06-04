function [estimatedSignal] = MyDAconverter(estimatedBitStream,Vp,N)
%% Construct a digital to analog converter     
%
% Inputs:
%       
%       estimatedBitStream: The signal vector which is not quantized 
%       Vp:                 The boundary [-V_p, V_p]
%       N:                  Bits
% 
% Outputs:
% 
%       estimatedSignal:    The bitStream generated from MyGrayCode function, or any other estimated gray coded bitStream    
% 
% Author: Farat Ghader Kourehpaz and Ghaith Ghadri
%
%% Script
% Create the first entry of "0" and "1"
arr = [];
arr = [arr, "0"];
arr = [arr, "1"];

% Every iteration of this loop generates 2*i codes from previously generated i codes.
i = 2;
j = 0;

while true
    if (i >= 2^N)
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

% Number of quantization levels
L = 2^N;

% Quantization step size
delta = (2*Vp) / (L);

% Quantization levels
Llow = -Vp+delta/2;
Lhigh = Vp-delta/2;
quantizationLevels = Llow:delta:Lhigh;
% Make it string
bitStream_str = string(estimatedBitStream);
% Save concatenated chars in an array 
gray_arr_reconst = [];

% Concatenate n chars at a time 
for i = 1:length(bitStream_str)
    if (mod(i, N) == 0)
        b = "";
        for j = N-1:-1:0
            b = b + bitStream_str(i-j); 
        end
        gray_arr_reconst = [gray_arr_reconst, b];
    else
        continue;
    end
end


% Replace the gray code with its respective decimal value found in
% quantizationLevels in the array estimatedSignal 
estimatedSignal = [];
for i = 1:length(gray_arr_reconst)
    for j = 1:length(arr)
        if (gray_arr_reconst(i) == arr(j))
            estimatedSignal(i) = quantizationLevels(j);
        else
            continue;
        end
    end
end
