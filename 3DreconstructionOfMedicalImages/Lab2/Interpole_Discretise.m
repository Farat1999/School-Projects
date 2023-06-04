function Y = Interpole_Discretise(X,n)
%Interpole_Discretise.
%
%   Y = Interpole_Discretise(X,n)
%
%  Interpolates by a cubic spline the points provided in the X input
%  matrix, then to discretize n points on this spline. The function thus 
%  returns a Y matrix of n points.  
%  Be careful, each LINE of X must contain the coordinates of a point.   
%

pp = spline(1:length(X),X');
Y = (ppval(pp,1:(length(X)-1)/(n-1):length(X)))';
