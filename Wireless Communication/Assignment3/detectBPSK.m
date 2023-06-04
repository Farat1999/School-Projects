function c=detectBPSK(r)
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
c = real(r)>0;
