function s=mapontoBPSK(c)
%% Map bitstream to constellation symbols
%
% Inputs:
%       
%       c:      The bitstream signal as column vector 
%      
% Outputs:
%       s:      The bitstream signal mapped to the BPSK constellation symbols (real values)
% 
% Author: Farat Ghader Kourehpaz and Ghaith Ghadri
%

% if c=0 => s=-1, and if c=1 => s=1
s = 2*c-1;