clc

%oldpath = path;
%path('C:\Users\Chris Cheang\MATLAB\Projects\Thrust Vect Mockup 1', oldpath)

%Functions = KinematicsFunctions;



% Resources

% Quaternion rotation: https://danceswithcode.net/engineeringnotes/quaternions/quaternions.html
% Rotation Matrix: https://mathworld.wolfram.com/RotationMatrix.html
% Quaternions in Matlab: https://uk.mathworks.com/help/robotics/ref/quaternion.html
% 3d plotting in Matlab: https://uk.mathworks.com/help/matlab/ref/plot3.html
% PID of mass-damper system in Matlab: https://ctms.engin.umich.edu/CTMS/index.php?example=Introduction&section=ControlPID

% Two degrees of freedom, thetaA and thetaB. A for the top bracket, B for the bracket connected to the engine. 
% Defining the two DoF this way to restrain the "roll" Dof of the engine
% around the center axes - important for finding actuator engine mount
% points and thus their length

thetaA = 0;    % right now these are opposite to the right hand rule
thetaB = 0;

% The origin is defined as the common pivot point of both DoF.

% Dimensions for the engine (currently defined not very rigorously but works)
% 4-2-2024: updated values for regen tvc, no changes to design required
% 28-10-2024: updated values for skywalker ver 1 tvc

rEngine = 32;  % radius of the actuator engine mounts
hTopRing = 23; % axial (z) distance downwards between the pivot point and the engine top ring (bottom edge)
hEngine = 166.9; % axial (z) distance downwards between the pivot point and the engine bottom
lPivot = 106.9; % axial (z) distance downwards between the pivot point and the engine actuator mount points
hMount = 5; % axial (z) distance upwards between the pivot point and the stationary actuator mount points. 17-2: before cut = 63, after should = 63-14.8
rMount = 203; % radius of the stationary actuator mounts, r=120
aMax = 10*pi/180; % maximum gimbal angle in radians
lead = 2; % lead of ball screw in mm

% 8-6-2024: Changing back to side mount from angle mount to accomodate legs
% note: don't put = 0 or else something breaks
mountangle = pi/4; %0.0001

h = 1; % i.e step size in time in s

m = 1;
mov(m) = getframe();

figure(1);


actALens = [];
actBLens = [];

actAClearances = [];
actBClearances = [];

thetaAs = [];
thetaAinverses = [];

nsteps = 64;

thetaGt = 0;
thetaRt = 1;

actARev = 0;
actBRev = 0;

eas = [0,0,0];  % cumulating the error values for the integral term
ebs = [0,0,0];  

actARevs = [0,0,0];
actBRevs = [0,0,0];

n = 0;




while n < 12 %n < 50 + nsteps + 1%150
    
    % This first section gives the target for the TVC to follow, can be given by mouse or set function. 

    %{
    %mouse version
    mouse = get( 0, 'PointerLocation' );
    x = 2*(mouse(1)/1383-0.5);
    y = 2*(mouse(2)/1383-0.15);

    if length(eas) > 10
        if sqrt(y^2+x^2) < aMax;
            thetaGt = sqrt(y^2+x^2);  %n/nsteps*
        else
            thetaGt = aMax;
        end
        thetaRt = atan2(y,x);
    end
    %}

    %{
    %set function version
    if n < 30
        thetaGt = 0.000001;
        thetaRt = pi/2;
        thetaRt = mod(thetaRt-pi,2*pi)-pi;
    elseif 30 <= n & n < 40
        thetaGt = aMax*(n-30)/10 + 0.000001;
        thetaRt = pi/2 + 0.000001;
        thetaRt = mod(thetaRt-pi,2*pi)-pi;
    elseif 40 <= n & n < 40 + nsteps
        thetaGt = aMax;
        thetaRt = pi/2 + 2*pi*(n-40)/nsteps+ 0.000001;
        thetaRt = mod(thetaRt-pi,2*pi)-pi;
    elseif 40 + nsteps <= n & n < 50 + nsteps
        thetaGt = aMax * (1 - (n - 40 - nsteps)/10);
        thetaRt = 5*pi/2;
        thetaRt = mod(thetaRt-pi,2*pi)-pi;
    else
        thetaGt = 0.0000001;
        thetaRt = 5*pi/2;
        thetaRt = mod(thetaRt-pi,2*pi)-pi;
    end

    pause(0.01)
    n = n + 1;
    %}

    % IMPORTANT NOTE!: It is probably a good idea to add some sort of error
    % clause, such that if tvcInverse or tvcForward returns an error, the
    % previous value is used instead.

    %actuatorTargetRevs = actuator_revs_from_polar(thetaGt,thetaRt,rEngine,lPivot,rMount,hMount,mountangle);

    %actARevt = actuatorTargetRevs(1);
    %actBRevt = actuatorTargetRevs(2);


    amax = 10;
    a = 0.6;

    actARevt = amax*sin(pi*(a*(n-2)+1)^2);
    actBRevt = -amax*cos(pi*(a*(n-2)+1)^2);

    n = n + 0.1;
    
    % note: removed PID simulating session, as ODrive takes care of all
    % that

    neutralLen = actuator_neutral(rEngine,lPivot,rMount,hMount,mountangle);
    actA = neutralLen + actARevt*lead;
    actB = neutralLen + actBRevt*lead;

    cartesian = cartesian_from_actuator_lengths(actA,actB,rEngine,lPivot,rMount,hMount,aMax,mountangle);
    thetaA = cartesian(1);
    thetaB = cartesian(2);
    
    draw(thetaA,thetaB,rEngine,hEngine,lPivot,rMount,hMount,hTopRing,mountangle);
    clearances = clearance(thetaA,thetaB,rEngine,hEngine,lPivot,rMount,hMount,hTopRing,mountangle);


    m = m + 1;


    actLens = actuator_lengths_from_cartesian(thetaA,thetaB,rEngine,lPivot,rMount,hMount,mountangle);

    actALen = actLens(1);
    actBLen = actLens(2);

    %actAClearnace = pointLineDist(pc1,[rMount,0,hMount],act1MountEngine);
    %actBClearnace = pointLineDist(pc2,[0,rMount,hMount],act2MountEngine);

    actALens = [actALens, actALen];
    actBLens = [actBLens, actBLen];

    actAClearances = [actAClearances, clearances(1)];
    actBClearances = [actBClearances, clearances(2)];
    
        
    angleA = round(thetaA*180/pi,2);
    angleB = round(thetaB*180/pi,2);

    anglePolarRad = cartesian_to_polar(thetaA, thetaB);
    anglePolar = round(anglePolarRad*180/pi,2);


    inverseAngles = cartesian_from_actuator_lengths(actALen,actBLen,rEngine,lPivot,rMount,hMount,aMax,mountangle);


    thetaAs = [thetaAs, thetaA];
    thetaAinverses = [thetaAinverses, inverseAngles(1)];

    
    % Testing inverse kinematics
    %disp("thetaA = " + angleA + ", thetaB = " + angleB + ", thetaGimbal = " + anglePolar(1) + ", thetaRoll = " + anglePolar(2) ...
        %+ ", ActA = " + actALen + ", ActB = " + actBLen + ", inv: thetaA = " + inverseAngles(1)*180/pi + ", thetaB = " + inverseAngles(2)*180/pi)

    % With motor rotation function - for testing
    neutralLength = actuator_neutral(rEngine,lPivot,rMount,hMount,mountangle);
    actARot = MotorActuatorRevolution(neutralLength,actALen);
    actBRot = MotorActuatorRevolution(neutralLength,actBLen);

    % Kinematics check - actARev and actBRev is what's drawn
    thetas = tvcForward(actARevt,actBRevt,rEngine,lPivot,rMount,hMount);
    nRots = tvcInverse(thetas(1),thetas(2),rEngine,lPivot,rMount,hMount);

    errors = nRots - [actARot, actBRot];
    error = errors(1) + errors(2);
    %disp("error = " + error)


    actARevs = [actARot, actARevs];
    actBRevs = [actBRot, actBRevs];


    % Bunch of different display options below

    %disp(actARot)
    %disp(actBRot)


    %disp("ActARot = " + actARot + ", ActBRot = " + actBRot + ". Reconverted: ActARot = " + nRots(1) + ", ActBRot = " + nRots(2))
    %disp("ActARot = " + actARot + ", ActBRot = " + actBRot + ". Converted thetas: thetaG = " + thetas(1) + ", thetaR = " + thetas(2) + ". Reconverted: ActARot = " + nRots(1) + ", ActBRot = " + nRots(2))

    disp("thetaGimbal = " + anglePolar(1) + ", thetaRoll = " + anglePolar(2))
    %disp("thetaA = " + angleA + ", thetaB = " + angleB + ", thetaGimbal = " + anglePolar(1) + ", thetaRoll = " + anglePolar(2) + ...
         %", ActA = " + actALen + ", ActB = " + actBLen + ", ActARot = " + actARot + ", ActBRot = " + actBRot)
    
    %disp("thetaGimbal = " + anglePolar(1) + ", thetaRoll = " + anglePolar(2) + ", ActA = " + actALen + ", ActB = " + actBLen + ", ActARot = " + actARot + ", ActBRot = " + actBRot)


    % testing polar to cartesian conversion
    %angleCartesianReconvert = polar_to_cartesian(anglePolarRad(1),anglePolarRad(2))*180/pi;
    %disp("thetaA = " + angleA + ", thetaB = " + angleB + ", thetaGimbal = " + anglePolar(1) + ", thetaRoll = " ...
        %+ anglePolar(2) + ", Reconversion check: thetaA = " + angleCartesianReconvert(1) + ", thetaB = " + angleCartesianReconvert(2))


    %mov(m) = getframe();

   
    

end

clearanceLimits = [min(actAClearances),min(actBClearances)];

actALenLimits = [min(actALens),max(actALens)];


actBLenLimits = [min(actBLens),max(actBLens)];

disp("Act A minClearance " + clearanceLimits(1) + ", Act B minClearances = " + clearanceLimits(2))

disp("Act A minLength = " + actALenLimits(1) + ", maxLength = " + actALenLimits(2))
disp("Act B minLength = " + actBLenLimits(1) + ", maxLength = " + actBLenLimits(2))

range = actALenLimits(2) - actBLenLimits(1);
disp("Range = " + range)

disp("Neutral Length = " + actuator_neutral(rEngine,lPivot,rMount,hMount,mountangle))
disp("Ideal Limits = " + actuator_limits(rEngine,lPivot,rMount,hMount,aMax,mountangle))

%inverseAngles = cartesian_from_actuator_lengths(369.9,350.2,rEngine,lPivot,rMount,hMount,aMax,mountangle);


%disp(actARevs)
%disp(actBRevs)

n = linspace(0,nsteps,nsteps+1);





%plot(n,thetaAs)
%hold on
%plot(n,thetaAinverses)
%hold off



% function for rotating a point around any line, defined by position vector pointCentre and direction vector 
function pointOut = rotate(pointIn,theta,pointCentre,vector)  % [xout,yout,zout] = rotate(xin,yin,zin,x0,y0,z0,theta,xvec,yvec,zvec)
    % Define rotation quaternion q and inverse
    q0 = cos(theta/2);
    q1 = vector(1)*sin(theta/2);
    q2 = vector(2)*sin(theta/2);
    q3 = vector(3)*sin(theta/2);
    q = quaternion(q0,q1,q2,q3);
    qmag = q0^2 + q1^2 + q2^2 + q3^2;
    qInv = quaternion(q0/qmag,-q1/qmag,-q2/qmag,-q3/qmag);

    %first create a point pointInTranslated that translates centre of
    %rotation pointCentre to the origin. pointCentre will be added back after all the rotation math stuff.
    pointInTranslated = pointIn - pointCentre;
    
    %define the point to be rotated into a quaternion
    p = quaternion(0,pointInTranslated(1),pointInTranslated(2),pointInTranslated(3));

    %output quaternion pPrime
    pPrime = qInv * p * q;
    
    %Extracting the coordinate from pPrime
    [~,xout,yout,zout] = parts(pPrime);

    %Add the pointCentre back
    pointOut = [xout,yout,zout] + pointCentre;
end
% Note: the pointCentre bit doesn't work yet

%
function pointOut = gimbal_cartesian(pointIn,A,B)
    aA = [1,0,0];  %axis A
    aB = rotate([0,1,0],A,[0,0,0],aA);  %axis B
    point = rotate(pointIn,A,[0,0,0],aA);
    pointOut = rotate(point,B,[0,0,0],aB);
end
%

%
function lengths = actuator_lengths_from_cartesian(A,B,rEngine,lPivot,rMount,hMount,mountangle)

    act1MountEngine = [rEngine*cos(mountangle),rEngine*sin(mountangle),-lPivot];
    act1MountEngine = gimbal_cartesian(act1MountEngine,A,B);
    lengths(1) = distance([rMount*cos(mountangle),rMount*sin(mountangle),hMount],act1MountEngine);

    act2MountEngine = [rEngine*cos(mountangle + pi/2),rEngine*sin(mountangle + pi/2),-lPivot];
    act2MountEngine = gimbal_cartesian(act2MountEngine,A,B);
    lengths(2) = distance([rMount*cos(mountangle + pi/2),rMount*sin(mountangle + pi/2),hMount],act2MountEngine);
end
%

%
function limits = actuator_limits(rEngine,lPivot,rMount,hMount,aMax,mountangle)
    %Consider the limits of actuator B as thetaA goes from -aMax to aMax - 90 degree ver (old)

    % 45 deg ver: consider actuator A
    min = polar_to_cartesian(aMax,mountangle);
    max = polar_to_cartesian(aMax,mountangle + pi);
    BFullRetract = actuator_lengths_from_cartesian(min(1),min(2),rEngine,lPivot,rMount,hMount,mountangle);
    BFullExtend = actuator_lengths_from_cartesian(max(1),max(2),rEngine,lPivot,rMount,hMount,mountangle);
    limits = [BFullRetract(1), BFullExtend(1)];
end
%

function length = actuator_neutral(rEngine,lPivot,rMount,hMount,mountangle)
    lengths = actuator_lengths_from_cartesian(0,0,rEngine,lPivot,rMount,hMount,mountangle);
    length = lengths(1);
end

%
function angles = cartesian_from_actuator_lengths(actA,actB,rEngine,lPivot,rMount,hMount,aMax,mountangle)
    function angles = cartesian_from_actuator_lengths_estimate(actA,actB,rEngine,lPivot,rMount,hMount,aMax,mountangle)
    
        %Uses and iterative method, based on a linear actuator length - gimbal
        %angle assumption for first guess
        limits = actuator_limits(rEngine,lPivot,rMount,hMount,aMax,mountangle);
        min = limits(1);
        max = limits(2);
        actRange = max - min;
        gradient = actRange/(2*aMax);
    
        %first estimate using a linear approximation + quadratic offset
        
    
        pA = [3/aMax,-actRange/(2*aMax),-actRange/2-min+actB-0.6];
        pB = [3/aMax,-actRange/(2*aMax),-actRange/2-min+actA-0.6];
    
        thetaAguess = roots(pA);
        thetaBguess = roots(pB);
    
        thetaA = thetaAguess(2);%2*aMax*(actB-min)/actRange - aMax;
        thetaB = - thetaBguess(2);%- (2*aMax*(actA-min)/actRange - aMax);  % for some reason I don't know about yet thetaB needs to be flipped
    
        angles = [thetaA,thetaB];
    end
    function F = root2d(x);  %x1 = thetaA, x2 = thetaB
        F = actuator_lengths_from_cartesian(x(1),x(2),rEngine,lPivot,rMount,hMount,mountangle) - [actA,actB];
    end
    fun = @root2d;
    options = optimoptions('fsolve','Display','Off');
    x0 = cartesian_from_actuator_lengths_estimate(actA,actB,rEngine,lPivot,rMount,hMount,aMax,mountangle);
    x = fsolve(fun,x0,options);
    
    angles = x;
end
%

function polarAngles = cartesian_to_polar(A, B)
    % Gimbal angle (polarAngles(1)) range = 0 < theta < pi/2, Roll angle
    % (polarAngles(2)) range = -pi < 0 < pi, 0 is x-axis
    point = gimbal_cartesian([0,0,-1],A,B);
    polarAngles(1) = atan2(norm(cross(point,[0,0,-1])), dot(point,[0,0,-1]));
    polarAngles(2) = atan2(point(2),point(1));
end

function cartesianAngles = unit_point_to_cartesian(point)
    thetaG = atan2(norm(cross(point,[0,0,-1])), dot(point,[0,0,-1]));
    thetaR = atan2(point(2),point(1));
    cartesianAngles = polar_to_cartesian(thetaG, thetaR);
end

function cartesianAngles = polar_to_cartesian(thetaG, thetaR)
    axes = [1,0,0];
    axes = rotate(axes,thetaR-pi/2,[0,0,0],[0,0,1]); % use this to define the axis of rotation for polar gimbal angle
    point = [0,0,-1];
    point = rotate(point,thetaG,[0,0,0],axes);
   
    N = [1,0,0];
    thetaB = pi/2 - atan2(norm(cross(point,N)), dot(point,N));

    pointProj = [0,point(2),point(3)];
    sign = - abs(thetaR)/thetaR;
    thetaA = sign * atan2(norm(cross(pointProj,[0,0,-1])), dot(pointProj,[0,0,-1]));

    cartesianAngles = [thetaA,thetaB];
end

function draw(thetaA,thetaB,rEngine,hEngine,lPivot,rMount,hMount,hTopRing,mountangle)
    % first define the first axis of rotation as a quaternion (thetaA, vectorA), which is always fixed by the top bracket
    axisA = [1,0,0];
    
    % Lists for parametric (choose)
    
    t = linspace(0,1,11);
    t2 = linspace(0,2*pi,31);


    % Line 1 = axisA, line 2 = axis B (rotated)
    
    X1 = 200*(2*t - 1)*axisA(1);
    Y1 = 200*(2*t - 1)*axisA(2);
    Z1 = 200*(2*t - 1)*axisA(3);
    
    %Axis B before rotation
    
    axisBP = [0,1,0];


    axisB = rotate(axisBP,thetaA,[0,0,0],axisA);
    
    X2 = 200*(2*t - 1)*axisB(1);
    Y2 = 200*(2*t - 1)*axisB(2);
    Z2 = 200*(2*t - 1)*axisB(3);
    
    %Line 3,4,5 : Circle representation
    
    %original position of circle with no rotation
    
    X3 = rEngine * cos(t2);
    Y3 = rEngine * sin(t2);
    Z3 = 0 * t2 - hTopRing;
    
    X4 = rEngine * cos(t2);
    Y4 = rEngine * sin(t2);
    Z4 = 0 * t2 - hEngine;

    X5 = rEngine * cos(t2);
    Y5 = rEngine * sin(t2);
    Z5 = 0 * t2 - lPivot;

    %Line 6: actuator 1 

    act1MountEngine = [rEngine*cos(mountangle),rEngine*sin(mountangle),-lPivot];
    act1MountEngine = gimbal_cartesian(act1MountEngine,thetaA,thetaB);

    X6 = rMount*cos(mountangle) - t * (rMount*cos(mountangle) - act1MountEngine(1));
    Y6 = rMount*sin(mountangle) - t * (rMount*sin(mountangle) - act1MountEngine(2));
    Z6 = hMount - t * (hMount - act1MountEngine(3));

    %Line 7: actuator 2 

    act2MountEngine = [rEngine*cos(mountangle + pi/2),rEngine*sin(mountangle + pi/2),-lPivot];
    act2MountEngine = gimbal_cartesian(act2MountEngine,thetaA,thetaB);

    X7 = rMount*cos(mountangle + pi/2) - t * (rMount*cos(mountangle + pi/2) - act2MountEngine(1));
    Y7 = rMount*sin(mountangle + pi/2) - t * (rMount*sin(mountangle + pi/2) - act2MountEngine(2));
    Z7 = hMount - t * (hMount - act2MountEngine(3));

    %Line 8: vertical axis

    verticalAxis = [0,0,-hEngine];
    verticalAxis = gimbal_cartesian(verticalAxis,thetaA,thetaB);

    X8 = 0 - t * (0 - verticalAxis(1));
    Y8 = 0 - t * (0 - verticalAxis(2));
    Z8 = 0 - t * (0 - verticalAxis(3));

    %calculating actuator clearance:

    % pc1 and pc2 = approximations of closest point between actuator A, B 
    % respectively and the top ring. This is acceptable, as the closest 
    % distance between an actuator and the top ring occurs when the other 
    % actuation angle is 0 (assuming the actuator stationary point is mounted
    % above the top ring , where the approximation is the actual closest point

    pc1 = [rEngine*cos(mountangle),rEngine*sin(mountangle),-hTopRing];
    pc1 = gimbal_cartesian(pc1,thetaA,thetaB);

    pc2 = [rEngine*cos(mountangle + pi/2),rEngine*sin(mountangle + pi/2),-hTopRing];
    pc2 = gimbal_cartesian(pc2,thetaA,thetaB);

    actAClearance = pointLineDist(pc1,[rMount,0,hMount],act1MountEngine);
    actBClearance = pointLineDist(pc2,[0,rMount,hMount],act2MountEngine);

    for i = 1:31   % gimbal each individual point of the circles
        outpoint = gimbal_cartesian([X3(i),Y3(i),Z3(i)],thetaA,thetaB);
        X3(i) = outpoint(1);
        Y3(i) = outpoint(2);
        Z3(i) = outpoint(3);

        outpoint = gimbal_cartesian([X4(i),Y4(i),Z4(i)],thetaA,thetaB);
        X4(i) = outpoint(1);
        Y4(i) = outpoint(2);
        Z4(i) = outpoint(3);

        outpoint = gimbal_cartesian([X5(i),Y5(i),Z5(i)],thetaA,thetaB);
        X5(i) = outpoint(1);
        Y5(i) = outpoint(2);
        Z5(i) = outpoint(3);
    end
    
    %Plots

    
  
    line1 = plot3(X1,Y1,Z1,'--','Color','b');
    
    hold on
    axis equal
    set(gca, 'Projection','perspective')
    xlim([-max([2*rEngine,1.5*rMount]),max([2*rEngine,1.5*rMount])]);
    ylim([-max([2*rEngine,1.5*rMount]),max([2*rEngine,1.5*rMount])]);
    zlim([-1.2*hEngine,abs(hMount)+0.3*rMount]);
    %view(0,0);
    
    origin = plot3(0,0,0);
    origin.Marker = "o";
    p1 = plot3(pc1(1),pc1(2),pc1(3));
    p1.Marker = "x";
    p2 = plot3(pc2(1),pc2(2),pc2(3));
    p2.Marker = "+";
    line2 = plot3(X2,Y2,Z2,'--');
    line3 = plot3(X3,Y3,Z3);
    line3 = plot3(X3,Y3,Z3);
    line4 = plot3(X4,Y4,Z4);
    line5 = plot3(X5,Y5,Z5);
    line6 = plot3(X6,Y6,Z6);
    line7 = plot3(X7,Y7,Z7);
    line8 = plot3(X8,Y8,Z8,'--');
    drawnow;
    
    
    hold off
end

function dist = clearance(thetaA,thetaB,rEngine,hEngine,lPivot,rMount,hMount,hTopRing,mountangle)

    act1MountEngine = [rEngine*cos(mountangle),rEngine*sin(mountangle),-lPivot];
    act1MountEngine = gimbal_cartesian(act1MountEngine,thetaA,thetaB);

    act2MountEngine = [rEngine*cos(mountangle + pi/2),rEngine*sin(mountangle + pi/2),-lPivot];
    act2MountEngine = gimbal_cartesian(act2MountEngine,thetaA,thetaB);

    %calculating actuator clearance:

    % pc1 and pc2 = approximations of closest point between actuator A, B 
    % respectively and the top ring. This is acceptable, as the closest 
    % distance between an actuator and the top ring occurs when the other 
    % actuation angle is 0 (assuming the actuator stationary point is mounted
    % above the top ring , where the approximation is the actual closest point

    pc1 = [rEngine*cos(mountangle),rEngine*sin(mountangle),-hTopRing];
    pc1 = gimbal_cartesian(pc1,thetaA,thetaB);

    pc2 = [rEngine*cos(mountangle + pi/2),rEngine*sin(mountangle + pi/2),-hTopRing];
    pc2 = gimbal_cartesian(pc2,thetaA,thetaB);

    actAClearance = pointLineDist(pc1,[rMount,0,hMount],act1MountEngine);
    actBClearance = pointLineDist(pc2,[0,rMount,hMount],act2MountEngine);

    dist = [actAClearance,actBClearance];
end

function dist = distance(pointA,pointB)
    dist = sqrt((pointA(1)-pointB(1))^2 + (pointA(2)-pointB(2))^2 + (pointA(3)-pointB(3))^2);
end

function dist = pointLineDist(pt, v1, v2)  %taken from https://uk.mathworks.com/matlabcentral/answers/95608-is-there-a-function-in-matlab-that-calculates-the-shortest-distance-from-a-point-to-a-line

    a = v1 - v2;%[v1(1)-v2(1),v1(2)-v2(2),v1(3)-v2(3)];
    b = pt - v2;%[pt(1)-v2(1),pt(2)-v2(2),pt(3)-v2(3)];
    dist = norm(cross(a,b)) / norm(a);
end

function nRotation = MotorActuatorRevolution(neutral_dis,actuator_dis)
    lead = 2;
    delta_dis = actuator_dis-neutral_dis;
    nRotation = delta_dis/lead;
end

function nRotations = actuator_revs_from_polar(thetaG,thetaR,rEngine,lPivot,rMount,hMount,mountangle)
    neutral_dis = actuator_neutral(rEngine,lPivot,rMount,hMount,mountangle);
    cartesianAngles = polar_to_cartesian(thetaG, thetaR);
    A = cartesianAngles(1);
    B = cartesianAngles(2);

    ActLens = actuator_lengths_from_cartesian(A,B,rEngine,lPivot,rMount,hMount,mountangle);
    ActALen = ActLens(1);
    ActBLen = ActLens(2);

    nRotations(1) = MotorActuatorRevolution(neutral_dis,ActALen);
    nRotations(2) = MotorActuatorRevolution(neutral_dis,ActBLen);
end
