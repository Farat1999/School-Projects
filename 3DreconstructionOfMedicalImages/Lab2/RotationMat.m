function [rMatHom] = RotationMat(angX, angY, angZ)
%% Rotation matrix formulation
%  Input:  x,y,z - axis rotation values
%  Output: 4x4 rotation matrix (homogenous coords)

rZhom = [cos(angZ) -sin(angZ) 0 0; sin(angZ) cos(angZ) 0 0; 0 0 1 0; 0 0 0 1];
rYhom = [cos(angY) 0 sin(angY) 0; 0 1 0 0; -sin(angY) 0 cos(angY) 0; 0 0 0 1];
rXhom = [1 0 0 0; 0 cos(angX) -sin(angX) 0; 0 sin(angX) cos(angX) 0; 0 0 0 1];

rMatHom = rZhom*rYhom*rXhom;
