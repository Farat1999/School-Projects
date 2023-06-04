function [SID, SOD, alpha, beta] = GetDicomFromCalib(K, R, T, DP)
%% Retrieve the DICOM parameters describing the view from the calibration 
%  parameters. 
%
%  Inputs:
%          K :      matrix of intrinsic parameters (3x3);
%          R :      rotation matrix (extrinsic) (3x3);
%          T :      translation vector (extrinsic) (3x1 or 1x3);
%          DP :     horizontal or vertical size of a pixel in mm;
%
%  Outputs:
%          SID :    distance from source to detector (image plane) in mm;
%          SOD :    distance from source to object (patient) in mm;
%          alpha :  primary angle in degrees;
%          beta :   secondary angle in degrees.
%
%  Notes:   
%       - the parameter DP is assumed to be fixed and provided as input;
%       - coordinate systems are assumed to be the same as in BuildViewGeom()
%       - the skew is assumed to be null ( K(1,2) == 0 )
%
%  Author : Philippe Debanné
%  Laboratoire LIV4D Lab
%  8-12-2014 (English translation: 19-10-2020)

% Force R matrix to be orthogonal :
[U,D,V] = svd(R);
D = eye(3,3);
R = U*D*V';

f = mean([K(1,1) K(2,2)]);  % Focal length (en pixels)
SID = f * DP;               % Source-detector distance (in mm)
SOD = norm(T);              % Source-object distance (in mm)

% alpha, beta : invert (transpose) R and decompose it into yaw,pitch,roll
% angles; we assume the form Rzyx(a,b,c) = Rz(a)*Ry(b)*Rx(c)
R = R';
% Assume that abs(b) <= PI/2 thus cos(b) >= 0
cb = sqrt(sum([R(1:2,1)' R(3,2:3)].^2)/2);
sb = -R(3,1);
% Angle around Y == b == Primary angle
alpha = atan2d(sb,cb); 
% Angle around X == c == Secondary angle
if abs(alpha) == 90,
    % Assume here that angle a (around Z) is ~= 0
    beta = -atan2d(-R(2,3), R(2,2));
else
    beta = -atan2d(R(3,2)/cosd(alpha), R(3,3)/cosd(alpha)); 
end

% TEST: angle around Z axis should be small (even negligible)
if abs(alpha) == 90,
    del = atan2d(R(2,3), R(1,3)); % gives (a - c)
    a = del + beta;
else
    a = atan2d(R(2,1)/cosd(alpha), R(1,1)/cosd(alpha));
end
fprintf(1,'\nGetDicomFromCalib(): Angle around Z axis = %f deg.\n\n',a);


