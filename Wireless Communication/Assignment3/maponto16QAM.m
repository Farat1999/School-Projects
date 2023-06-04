function s = maponto16QAM(c)
%% Map bitstream to constellation symbols
%
% Inputs:
%       
%       c:      The bitstream signal as column vector 
%      
% Outputs:
%       s:      The bitstream signal mapped to the 16QAM constellation symbols (complex values)
% 
% Author: Farat Ghader Kourehpaz and Ghaith Ghadri
%

% make it row vector
c = c(:)';

% 16 symbols
M =16;

% Number of bits per symbol is log2(M)
k = log2(M);

% Scaling factor 
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

% Bitstream to 4-bit gray code
symbols = [];
% Concatenate k chars at a time 
for i = 1:length(c)
    if (mod(i, k) == 0)
        b = "";
        for j = k-1:-1:0
            b = b + c(i-j); 
        end
        symbols = [symbols, b];
    else
        continue;
    end
end

% Replace the symbols with its respective imaginary value 
s = [];
for i = 1:length(symbols)
    for j = 1:length(arr)
        if (symbols(i) == arr(j))
            s(i) = constellation(j);
        else
            continue;
        end
    end
end

% Make it to column vector
s = reshape(s, length(s), []);


