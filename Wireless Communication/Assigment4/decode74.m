function bEst=decode74(c)

% Reshape the column vector to row vector
c = reshape(c, [], length(c));

% Encode a (7,4) Hamming code
n = 7;
k = 4;
m = n-k;

% Create the parity check matrix H
[H, ~] = hammgen(m);

% Calculate the syndrome
s = mod(c*H', 2); % s is a row vector of length (n-k)

% Check if syndrome is all zeros
if sum(s) == 0
    c_corrected = c(n-k+1:end); % extract last k bits
    error_loc = []; % no errors detected
else
    % Convert the syndrome to decimal to identify the error location
    error_loc = bi2de(s, 'left-msb') + 1; % add 1 to match MATLAB indexing
    % Correct the error
    c(error_loc) = mod(c(error_loc) + 1, 2);
    c_corrected = c(n-k+1:end); % extract last k bits
end

bEst = c_corrected;

bEst = reshape(bEst, length(bEst), []);