clc,clearvars
close all;
clear;

% Resources

% Quaternion rotation: https://danceswithcode.net/engineeringnotes/quaternions/quaternions.html
% Rotation Matrix: https://mathworld.wolfram.com/RotationMatrix.html
% Quaternions in Matlab: https://uk.mathworks.com/help/robotics/ref/quaternion.html
% 3d plotting in Matlab: https://uk.mathworks.com/help/matlab/ref/plot3.html

% Two degrees of freedom, thetaA and thetaB. A for the top bracket, B for the bracket connected to the engine. 

thetaA = 0;
thetaB = 0;

% The origin is defined as the common pivot point of both DoF.


% first define the first axis of rotation as a quaternion (thetaA, vectorA), which is always fixed by the top bracket
axisA = [1,0,0];

% Lists for parametric (choose)

t = linspace(-200,200,11);
t2 = linspace(0,2*pi,101);

% Line 1 = axisA, 

X1 = t*axisA(1);
Y1 = t*axisA(2);
Z1 = t*axisA(3);

%Axis B before rotation

axisBPreRot = [0,1,0];


m = 1;
mov(m) = getframe();

figure(1);

for n = 1:100
    
    axisB = rotate(axisBPreRot,thetaA,[0,0,0],axisA);
    
    X2 = t*axisB(1);
    Y2 = t*axisB(2);
    Z2 = t*axisB(3);
    
    %Line 3: Circle representation
    
    %original position of circle with no rotation
    
    X3 = 100 * cos(t2);
    Y3 = 100 * sin(t2);
    Z3 = 0 * t2 - 55;
    
    for i = 1:101
        %first rotation by thetaA around axisA
        outpoint1 = rotate([X3(i),Y3(i),Z3(i)],thetaA,[0,0,0],axisA);
        X3(i) = outpoint1(1);
        Y3(i) = outpoint1(2);
        Z3(i) = outpoint1(3);
    
        %second rotation by thetaB around axisB
        outpoint2 = rotate([X3(i),Y3(i),Z3(i)],thetaB,[0,0,0],axisB);
        X3(i) = outpoint2(1);
        Y3(i) = outpoint2(2);
        Z3(i) = outpoint2(3);
    end
    
    %Plots

    axis equal
    
    line1 = plot3(X1,Y1,Z1);
    xlim([-200,200]);
    ylim([-200,200]);
    zlim([-200,200]);
    hold on
    line2 = plot3(X2,Y2,Z2);
    line3 = plot3(X3,Y3,Z3);
    drawnow;
    
    hold off

    %set(gca,'XLim',[-200 200],'YLim',[-200 200],'ZLim',[-200 200]);


    m = m + 1;
    thetaA = 0.4*sin(0.1*n)
    thetaB = 0.4*cos(0.1*n)
    mov(m) = getframe();

end






% function for rotating a point around any line, defined by position vector pointCentre and direction vector 
function [pointOut] = rotate(pointIn,theta,pointCentre,vector)  % [xout,yout,zout] = rotate(xin,yin,zin,x0,y0,z0,theta,xvec,yvec,zvec)
    % Define rotation quaternion q and inverse
    q0 = cos(theta/2);
    q1 = vector(1)*sin(theta/2);
    q2 = vector(2)*sin(theta/2);
    q3 = vector(3)*sin(theta/3);
    q = quaternion(q0,q1,q2,q3);
    qmag = q0^2 + q1^2 + q2^2 + q3^2;
    qInv = quaternion(q0/qmag,-q1/qmag,-q2/qmag,-q3/qmag);

    %first create a point pointInTranslated that translates centre of
    %rotation pointCentre to the origin. pointCentre will be added back after all the rotation math stuff.
    pointInTranslated = pointIn - pointCentre;
    
    %define the point to be rotated into a quaternion
    p = quaternion(0,pointIn(1),pointIn(2),pointIn(3));

    %output quaternion pPrime
    pPrime = qInv * p * q;
    
    %Extracting the coordinate from pPrime
    [~,xout,yout,zout] = parts(pPrime);

    %Add the pointCentre back
    pointOut = [xout,yout,zout] + pointCentre;
end

% Note: the pointCentre bit doesn't work yet

