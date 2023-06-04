function bEst = decode1511(c)
% The number of message bits and code word length
n = 15;
k = 11;

% Generate the parity check matrix H
H = hammgen(n-k);

% Ensure that the length of c is an integer multiple of n
num_codewords = ceil(length(c)/n);
c = [c zeros(1, num_codewords*n - length(c))];

% Reshape the input vector c into a matrix with num_codewords rows
c_matrix = reshape(c,n,num_codewords)';

% Initialize the decoded message matrix
decoded_message = zeros(num_codewords,k);

% Loop over all code words and decode them
for i = 1:num_codewords
    % Extract the i-th code word
    c_i = c_matrix(i,:);
    
    % Compute the syndrome
    syndrome = mod(c_i*H',2);
    
    % If the syndrome is non-zero, correct the error
    if any(syndrome)
        % Find the column of H that corresponds to the error
        [~,error_col] = ismember(syndrome,H,'rows');
        
        % Flip the error bit
        c_i(error_col) = mod(c_i(error_col) + 1, 2);
    end
    
    % Extract the message bits from the corrected code word
    decoded_message(i,:) = c_i(1:k);
end

% Reshape the decoded message into a vector
bEst = reshape(decoded_message',1,[]);
end
