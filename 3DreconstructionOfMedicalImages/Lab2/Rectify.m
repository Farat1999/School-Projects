function [T1,T2,Pn1,Pn2] = Rectify(Po1,Po2)
% RECTIFY: compute rectification matrices
%
% Input:
%           Po1 : 3x4 projection matrix of 1st source
%           Po2 : 3x4 projection matrix of 2nd source
%
% Output:
%           T1  : rectifying transformation for 1st image (3x3)
%           T2  : rectifying transformation for 2nd image (3x3)
%           Pn1 : new projection matrix for 1st source (3x4)
%           Pn2 : new projection matrix for 2nd source (3x4)
%
%
% Author: Andrea Fusiello, University of Udine, Italy
% Date: 2000-03-17
%
% Source: http://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/FUSIELLO2/node5.html
% Reference: A. Fusiello, Tutorial on Rectification of Stereo Images, Nov. 1999, available in
%   CVOnline: On-line Compendium of Computer Vision, (R. Fisher, ed.)
%   (http://homepages.inf.ed.ac.uk/rbf/CVonline/) and on ResearchGate.
%

% factorize old PPMs
[A1,R1,~] = art(Po1);
[A2,~,~]  = art(Po2);

% optical centers (unchanged)
c1 = - inv(Po1(:,1:3))*Po1(:,4);
c2 = - inv(Po2(:,1:3))*Po2(:,4);

% new x axis (= direction of the baseline)
v1 = (c1-c2);
% new y axes (orthogonal to new x and old z)
v2 = cross(R1(3,:)',v1);
% new z axes (orthogonal to baseline and y)
v3 = cross(v1,v2);

% new extrinsic parameters
R = [v1'/norm(v1)
    v2'/norm(v2)
    v3'/norm(v3)];
% translation is left unchanged

% new intrinsic parameters (arbitrary)
A = (A1 + A2)./2;
A(1,2)=0; % no skew

% new projection matrices
Pn1 = A * [R -R*c1 ];
Pn2 = A * [R -R*c2 ];

% rectifying image transformation
T1 = Pn1(1:3,1:3)/Po1(1:3,1:3); % *inv(Po1(1:3,1:3));
T2 = Pn2(1:3,1:3)/Po2(1:3,1:3); % *inv(Po2(1:3,1:3));

end

function [A,R,t] = art(P)
% ART: factorize a PPM as  P=A*[R;t]
% 
Q = inv(P(1:3, 1:3));
[U,B] = qr(Q);

R = inv(U);
t = B*P(1:3,4);
A = inv(B);
A = A ./A(3,3);

end
 