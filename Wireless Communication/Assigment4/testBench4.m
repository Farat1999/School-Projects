%% decode1511
% Hamming(7,4) code parameters
n = 7;      % length of codewords
k = 4;      % number of message bits
d = 3;      % minimum Hamming distance

% define the generator and parity matrix
[H, G] = hammgen(n-k);

% received codeword
r = [1 1 0 0 1 0 1 0 0 1 0 0 1 0];

% calculate the syndrome
syndrome = mod(r*H', 2);

% find the error pattern
errorPattern = zeros(1, n);
for i = 1:n
    % create a test vector with a single 1 in the i-th position
    testVector = zeros(1, n);
    testVector(i) = 1;
    
    % calculate the syndrome for the test vector
    testSyndrome = mod(testVector*H', 2);
    
    % if the test syndrome matches the actual syndrome, then the i-th bit is
    % the location of the error
    if isequal(testSyndrome, syndrome)
        errorPattern(i) = 1;
    end
end

% correct the error
decodedCodeword = mod(r + errorPattern, 2);

% extract the message bits from the decoded codeword
decodedMessage = decodedCodeword(1:k);
