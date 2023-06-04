function [source, detector] = BuildViewGeom(SID, SOD, DP, alpha, beta, imSize)
%% Calculate the angiographic imaging geometry from the DICOM parameters describing 
% the view. 
%
%  Inputs:
%          SID :    distance from source to detector (image plane) in mm;
%          SOD :    distance from source to object (patient) in mm;
%          DP :     horizontal or vertical size of a pixel in mm;
%          alpha :  primary angle in degrees;
%          beta :   secondary angle in degrees;
%          imSize : vector of 2 integers giving image size in pixels.
%
%  Outputs:
%          source :    structure containing the computed geometry of the
%                       source and the projection parameters,
%          detector :  structure containing the computed geometry of the
%                       imaging plane.
%
%  Note:   The patient lies on their back along the Y-axis of the world
%          coordinate system. The X-axis is to their left and they look in the
%          direction of the Z-axis.
%
%  Authour : Mitchel Benovoy
%  Modified : Philippe Debanné
%  Laboratoire LIV4D Lab
%  21-10-2014  (English translation: 19-10-2020)

source = [];
detector = [];

primAngle = (alpha)*pi/180;
secAngle = (beta)*pi/180;

% X-Ray source

% Extrinsic parameters :
% Translation matrix :
cMat = TranslationMat(0,0,-SOD);
% Rotation matrix :
rotMat = RotationMat(-secAngle,primAngle,0);
% Geometric transformation expressing source ref. frame with respect to
% world ref. frame :
rcMat = rotMat * cMat;
% Extrinsic matrix = inverse of previous one (expresses world ref. frame
% with respect to source ref. frame) :
extMat = inv(rcMat); extMat(end,:) = [];

% Intrinsic parameters :
u0 = imSize(1)/2;             % Principal point (horiz. coord.)
v0 = imSize(2)/2;             % Principal point (vert. coord.)
f = SID / DP;                 % Focal length (in pixels)
K = [f 0 u0;0 f v0;0 0 1];    % Intrinsic matrix

% Source's projection matrix :
P = K*extMat;

% Store parameters in a structure :
source.u0 = u0;
source.v0 = v0;
source.f = f;
source.T = extMat(1:3,end);
source.R = extMat(1:3,1:3);
source.K = K;
source.extMat = extMat;
source.P = P;
source.worldPos = rcMat(1:3,end);
source.sid = SID;
source.sod = SOD;
source.angles = [primAngle secAngle];

% X-Ray detector (image plane)

% Extrinsic parameters :
% Translation matrix :
cMat = TranslationMat(0,0,SID - SOD);
% Rotation matrix :
rotMat = RotationMat(-secAngle,primAngle,0);
% Geometric transformation expressing detector ref. frame with respect to
% world ref. frame : 
rcMat = rotMat * cMat;

% Store parameters in a structure :
detector.worldPos = rcMat(1:3,end);

