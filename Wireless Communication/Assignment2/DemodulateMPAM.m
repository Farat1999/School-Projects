function [estimatedBitstream, BER] = DemodulateMPAM(receivedSignal,M,Es,transmittedBitstream, matchedFilterFlag)
% Demodulate an analog signal and reconstruct the bitstream
% 
% Inputs:
%
%   receivedSignal:         An analog signal with decimal values
%   M:                      Number of amplitude levels, where M = 2^k
%   Es:                     Average symbol energy
%   transmittedBitstream:   The true bitstream that is supposed to be reconstructed
% 
% Output:
%
%   estimatedBitstream:     The reconstructed bitstream
%   BER:                    Bit Error Rate
% 
% Authors: Farat Ghader Kourehpaz and Ghaith Ghadri



% Sample the received signal T = 100
T = 100;
%samples = T:T:length(receivedSignal);
%sampledSignal = receivedSignal(samples);

% Number of bits
k = log2(M);

% Find the equidistant distance d between each symbol 
i = 1:1:M;
d = sqrt((Es*M)/sum((2*i-1-M).^2));

% The amplitude levels
Ai = (2*i-1-M)*d;

if matchedFilterFlag == 1
    filter_signal = zeros(1, length(receivedSignal));
    matchedfilter = ones(1, 100);

    for i = 1:100:length(receivedSignal) - 99
        filter_signal(i:i+99) = conv(receivedSignal(i:i+99), matchedfilter, 'same')*0.01;
    end
    final_signal = filter_signal(50:T:end);
else
    final_signal = receivedSignal(50:T:end);
end

% Define the decision regions Z_1, Z_2, ..., Z_M
Z = [];
for i = 1:1:length(final_signal)
    for j = 1:1:length(Ai)
        if (final_signal(i) < Ai(1)-d)
            Z(i) = Ai(1);
        end
        if (final_signal(i)>=Ai(j)-d && final_signal(i)< Ai(j)+d) 
            Z(i) = Ai(j);
        end
        if (final_signal(i) > Ai(end)+d)
            Z(i) = Ai(end);
        end
    end
end

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


% Match signal level with gray code
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
estimatedBitstream = str2double(gray_arr);

% Length of the bitstream is uneven 
if mod(length(transmittedBitstream), k) ~= 0
     % remove the last bit from zero-padding in M-PAM
     estimatedBitstream = estimatedBitstream(1:end - 1); 
end

% Compute the Bit Error Rate
errors = sum(estimatedBitstream ~= transmittedBitstream);
BER = errors/length(transmittedBitstream);
