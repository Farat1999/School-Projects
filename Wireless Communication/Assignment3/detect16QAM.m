function c=detect16QAM(r)
%% Map bitstream to constellation symbols
%
% Inputs:
%       
%       s:      The signal (complex values) mapped to the correct bits
%      
% Outputs:
% 
%       c:      The detected bitstream signal as column vector
% 
% Author: Farat Ghader Kourehpaz and Ghaith Ghadri
%

% Number of bits
k = log2(16);

% Calculated equidistant 2d between each symbol 
d = sqrt(1/10);

% Constellation values starting from 0000 and ending 1000
constellation = [-3-3i, -1-3i, 1-3i, 3-3i, ...
    3-1i, 1-1i, -1-1i, -3-1i, ...
    -3+1i, -1+1i, 1+1i, 3+1i, ...
    3+3i, 1+3i, -1+3i, -3+3i].*d;

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

% Find the minimum distance between r and constellation symbol
[~, idx_min] = min(abs(r(:)-constellation), [], 2);

% Match with correct gray value
c_gray = [];
for i = 1:length(idx_min)
    c_gray = [c_gray, arr(idx_min(i))];
end

% Convert to bitstream
c_gray = split(c_gray, "", 2);
% Remove empty cells
c_gray = c_gray(c_gray~="")';
% Change string to double in array
c = str2double(c_gray);

% Reshape c to column vector
c = c(:);
