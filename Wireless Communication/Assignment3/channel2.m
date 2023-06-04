function noisySignal = channel2(transmittedSignal,noisePower)
%% Add noise
%
% Inputs:
%       
%       transmittedSignal:  The signal as complex values
%       noisePower:         The variance
%      
% Outputs:
%       noisySignal:        The signal as complex values with added noise
% 
% Author: Farat Ghader Kourehpaz and Ghaith Ghadri
%

% Convert it to row vector
transmittedSignal = reshape(transmittedSignal, [], length(transmittedSignal));

% Divide by 2 and take the square root of noise power so that the real and 
% imaginary parts each contribute to one half of the variance
noise_re = sqrt(noisePower/2)*randn(1, length(transmittedSignal)); % Real part of the noise
noise_im = sqrt(noisePower/2)*randn(1, length(transmittedSignal)); % Imaginary part of the noise

% Combine the real and imaginary part
noise = noise_re + 1i*noise_im; 

% Add noise to the signal
noisySignal = transmittedSignal + noise;

% Make it to column vector
noisySignal = reshape(noisySignal, length(noisySignal), []);