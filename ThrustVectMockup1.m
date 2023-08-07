clc,clearvars

% Resources

% Quaternion rotation: https://danceswithcode.net/engineeringnotes/quaternions/quaternions.html
% Rotation Matrix: https://mathworld.wolfram.com/RotationMatrix.html
% Quaternions in Matlab: https://uk.mathworks.com/help/robotics/ref/quaternion.html
% 3d plotting in Matlab: https://uk.mathworks.com/help/matlab/ref/plot3.html

% Two degrees of freedom, thetaA and thetaB. A for the top bracket, B for the bracket connected to the engine. 
% The origin is defined as the common pivot point of both DoF.
% first define the first axis of rotation as a quaternion (thetaA, , which is always fixed by the top bracket

thetaA = 0;
thetaB = 0

axisA = quaternion()
