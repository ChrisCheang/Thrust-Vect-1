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

t = linspace(0,1,11);
t2 = linspace(0,2*pi,101);

% Line 1 = axisA, line 2 = axis B (rotated)

X1 = 200*(2*t - 1)*axisA(1);
Y1 = 200*(2*t - 1)*axisA(2);
Z1 = 200*(2*t - 1)*axisA(3);

%Axis B before rotation

axisBP = [0,1,0];


m = 1;
mov(m) = getframe();

figure(1);

for n = 1:101
    
    
    axisB = rotate(axisBP,thetaA,[0,0,0],axisA);
    
    X2 = 200*(2*t - 1)*axisB(1);
    Y2 = 200*(2*t - 1)*axisB(2);
    Z2 = 200*(2*t - 1)*axisB(3);
    
    %Line 3,4 : Circle representation
    
    %original position of circle with no rotation
    
    X3 = 100 * cos(t2);
    Y3 = 100 * sin(t2);
    Z3 = 0 * t2 - 55;
    
    X4 = 100 * cos(t2);
    Y4 = 100 * sin(t2);
    Z4 = 0 * t2 - 200;

    %Line 5: axis B without rotation

    X5 = 200*t*axisBP(1);
    Y5 = 200*t*axisBP(2);
    Z5 = 200*t*axisBP(3);

    %Line 6: actuator 1 

    act1MountEngine = [100*cos(0),100*sin(0),-200];
    act1MountEngine = rotate([act1MountEngine(1),act1MountEngine(2),act1MountEngine(3)],thetaA,[0,0,0],axisA);
    act1MountEngine = rotate([act1MountEngine(1),act1MountEngine(2),act1MountEngine(3)],thetaB,[0,0,0],axisB);

    X6 = 200 - t * (200 - act1MountEngine(1));
    Y6 = 0 - t * (0 - act1MountEngine(2));
    Z6 = 55 - t * (55 - act1MountEngine(3));

    %Line 7: actuator 2 

    act2MountEngine = [100*cos(pi/2),100*sin(pi/2),-200];
    act2MountEngine = rotate([act2MountEngine(1),act2MountEngine(2),act2MountEngine(3)],thetaA,[0,0,0],axisA);
    act2MountEngine = rotate([act2MountEngine(1),act2MountEngine(2),act2MountEngine(3)],thetaB,[0,0,0],axisB);

    X7 = 0 - t * (0 - act2MountEngine(1));
    Y7 = 200 - t * (200 - act2MountEngine(2));
    Z7 = 55 - t * (55 - act2MountEngine(3));

    %Line 8: vertical axis

    verticalAxis = [0,0,-200];
    verticalAxis = rotate([verticalAxis(1),verticalAxis(2),verticalAxis(3)],thetaA,[0,0,0],axisA);
    verticalAxis = rotate([verticalAxis(1),verticalAxis(2),verticalAxis(3)],thetaB,[0,0,0],axisB);

    X8 = 0 - t * (0 - verticalAxis(1));
    Y8 = 0 - t * (0 - verticalAxis(2));
    Z8 = 0 - t * (0 - verticalAxis(3));

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

        %first rotation by thetaA around axisA
        outpoint1 = rotate([X4(i),Y4(i),Z4(i)],thetaA,[0,0,0],axisA);
        X4(i) = outpoint1(1);
        Y4(i) = outpoint1(2);
        Z4(i) = outpoint1(3);
    
        %second rotation by thetaB around axisB
        outpoint2 = rotate([X4(i),Y4(i),Z4(i)],thetaB,[0,0,0],axisB);
        X4(i) = outpoint2(1);
        Y4(i) = outpoint2(2);
        Z4(i) = outpoint2(3);

    end
    
    %Plots
  
    line1 = plot3(X1,Y1,Z1,'--','Color','b');
    axis equal
    set(gca, 'Projection','perspective')
    xlim([-200,200]);
    ylim([-200,200]);
    zlim([-500,200]);
    hold on
    line2 = plot3(X2,Y2,Z2,'--');
    line3 = plot3(X3,Y3,Z3);
    line4 = plot3(X4,Y4,Z4);
    %line5 = plot3(X5,Y5,Z5);
    line6 = plot3(X6,Y6,Z6);
    line7 = plot3(X7,Y7,Z7);
    line8 = plot3(X8,Y8,Z8,'--');
    drawnow;
    
    hold off

    %set(gca,'XLim',[-200 200],'YLim',[-200 200],'ZLim',[-200 200]);


    m = m + 1;

    if n<50
        thetaA = 0.4*sin(0.04*n*pi);
    else 
        thetaB = 0.4*sin(0.04*n*pi);
    end

    actALen = distance([200,0,55],act1MountEngine);
    actBLen = distance([0,200,55],act2MountEngine);
        
    thetaA
    thetaB


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

function dist = distance(pointA,pointB)
    dist = sqrt((pointA(1)-pointB(1))^2 + (pointA(2)-pointB(2))^2 + (pointA(3)-pointB(3))^2);
end
% Note: the pointCentre bit doesn't work yet

