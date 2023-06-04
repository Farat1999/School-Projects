function c=encodeHAM1511(b)
% Reshape the column vector to row vector
b = reshape(b, [], length(b));

% Encode a (15,11) Hamming code
n = 15;
k = 11;
m = n-k;

% Generate the generator matrix G for the (15,11) Hamming code where n-k=4
[~, G] = hammgen(m);

% Multiply the message bits by the generator matrix G to produce the code word
% Use instead of XOR operator
c = mod(b*G,2);

c = reshape(c, length(c), []);