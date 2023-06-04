function [bitStream] = MyGraycode2(quantizedSignal,Vp,N)
%% Generate n-bit gray code
% Inputs:
%       
%       quantizedsignal:    The signal vector which is quantized 
%       Vp:                 The boundary [-V_p, V_p]
%       N:                  Number of bits
% 
% Outputs:
% 
%       bitStream:          Transformed quantized signal to gray code
%
% This algorithm was inspired by https://www.geeksforgeeks.org/generate-n-bit-gray-codes/ 
%
% Authors: Farat Ghader Kourehpaz and Ghaith Ghadri
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
 
% Create an array to save the split gray code values
gray_arr = [];

if (N>1)
    % Number of quantization levels
    L = 2^N;
    
    % Quantization step size
    delta = (2*Vp) / (L);
    
    % Quantization levels
    Llow = -Vp+delta/2;
    Lhigh = Vp-delta/2;
    quantizationLevels = Llow:delta:Lhigh;
    
    % replace signal value with it's respective gray code 
    % Remember that the indexes of the quantization level play a key role here
    for i = 1:length(quantizedSignal)
        for j = 1:length(quantizationLevels)
            if (quantizedSignal(i) == quantizationLevels(j))
                gray_arr = [gray_arr, arr(j)];
            end
        end
    end
    
    % Split string
    gray_arr = split(gray_arr, "", 2);
    % Remove empty cells
    gray_arr = gray_arr(gray_arr~="");
    % Change string to double in array
    bitStream = str2double(gray_arr');
else
    % If n = 1, bitStream = [0, 1]
    bitStream = str2double(arr);
end