function [tMatHom] = TranslationMat(dX, dY, dZ)
%% Translation matrix formulation
%  Input:  x,y,z translation values
%  Output: 4x1 translation matrix (homogenous coords)

tMatHom = eye(4);
tMatHom(1:3,4) = [dX;dY;dZ];
