function transmitSignal = MyMPAM(bitstream,M,Es)
% Convert a bit stream into an M-ary baseband PAM-signal
% 
% Inputs:
%   bitstream: input bit stream as a vector of ones and zeroes
%   M: number of amplitude levels, where M = 2^k
%   Es: average symbol energy
% Output:
%   transmitSignal: the M-ary baseband PAM-signal
% 
% Authors: Farat Ghader Kourehpaz and Ghaith Ghadri

% Number of bits
k = log2(M);

% Find the equidistant distance d between each symbol 
i = 1:1:M;
d = sqrt((Es*M)/sum((2*i-1-M).^2));

% The amplitude levels
Ai = (2*i-1-M)*d;

% Number of symbols in the bitstream
N = floor(length(bitstream)/k);

% Length of the bitstream is uneven 
if mod(length(bitstream), k) ~= 0
    % Zero-pad the bitstream to ensure that it contains a whole number of messages
    bitstream = [bitstream, zeros(1, k - mod(length(bitstream), k))];
    N = N + 1;
end

% Reshape the bitstream to a matrix
bitstream_reshape = reshape(bitstream',k, N)';

% Create a gray-code
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

% Match the bits with amplitude levels with the help of the gray-code
pam_symbols = zeros(size(bitstream_reshape, 1),1);
for i=1:size(bitstream_reshape, 1)
    row = num2str(bitstream_reshape(i, :));
    row(row == ' ') = ''; % remove spaces
    idx = find(strcmp(arr, row));
    pam_symbols(i) = Ai(idx);
end

% transmitSignal = pam_symbols';
transmitSignal = rectpulse(pam_symbols', 100);
