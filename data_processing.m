%clc,clearvars
clc
%close all;
%clear;

rEngine = 90.168;  % radius of the actuator engine mounts
hTopRing = 55; % axial (z) distance downwards between the pivot point and the engine top ring (bottom edge)
hEngine = 298; % axial (z) distance downwards between the pivot point and the engine bottom
lPivot = hEngine; % axial (z) distance downwards between the pivot point and the engine actuator mount points
hMount = 65; % axial (z) distance upwards between the pivot point and the stationary actuator mount points
rMount = 180; % radius of the stationary actuator mounts, r=120
aMax = 10*pi/180; % maximum gimbal angle in radians
lead = 4; % lead of ball screw in mm

%
%read csv (import data first - put into thrust vect mockup 1 folder, then right click to import data)
D = tvctest3252;
M = table2array(D);

%divide shadowcount by 8192 
resolution = 8192;

%

A = zeros(length(M(:,1))-1,11);  
% 1 time, 2 thetaG, 3 thetaR, 4 current0, 5 current1, 6 torque0, 7 torque1
% 8 force0, 9 force1, 10 turns0, 11 turns1, 12 speed0, 13 speed1

for t = 1:length(M(:,1))-1
    thetas = tvcForward(M(t,3)/resolution,M(t,4)/resolution,rEngine,lPivot,rMount,hMount);
    A(t,1) = (M(t,1) - M(1,1)) / 10^9 ;
    A(t,2) = 180*thetas(1)/pi; % motor 0
    A(t,3) = 180*thetas(2)/pi; % motor 1
    A(t,4) = M(t,5); % current 0
    A(t,5) = M(t,6); % current 1
    A(t,10) = M(t,3)/resolution;  % turns 0
    A(t,11) = M(t,4)/resolution;  % turns 1
end
%

% smoothing current data
A(:,4) = smoothdata(A(:,4),"movmean",10);
A(:,5) = smoothdata(A(:,5),"movmean",10);


R = ones(length(A(:,1))-1,3); % horizontal and vertical resolved angles with 20 deg offset
for t = 1:length(M(:,1))-1
    R(t,1) = A(t,1);
    R(t,2) = -A(t,2)*cos(A(t,3)*pi/180); % x component
    R(t,3) = -A(t,2)*sin(A(t,3)*pi/180) + 20; % y component with 20 deg offset
end


% Torques
Kv = 140;
Kv = Kv*2*pi/60;
Kt = 1/Kv;

A(:,6) = A(:,4)*Kt; % torque0
A(:,7) = A(:,5)*Kt; % torque1


% calculating velocities
len = length(A(:,1))-1;
A(1,12) = (A(2,10) - A(1,10)) / (A(2,1)-A(1,1)); % forward divided difference for first point
A(1,13) = (A(2,11) - A(1,11)) / (A(2,1)-A(1,1)); 
for t = 2:len-1
    A(t,12) = (A(t+1,10) - A(t-1,10)) / (A(t+1,1) - A(t-1,1)); % central divided difference for middle points
    A(t,13) = (A(t+1,11) - A(t-1,11)) / (A(t+1,1) - A(t-1,1));
end
A(len,12) = (A(len,10) - A(len-1,10)) / (A(len,1)-A(len-1,1)); % backward divided difference for last point
A(len,13) = (A(len,11) - A(len-1,11)) / (A(len,1)-A(len-1,1));
% 



figure
hold on
box on
grid on
xlabel('time(s)');

% torque
%
ylabel('smoothed torque (Nm)');
plot(A(:,1),A(:,6));
plot(A(:,1),A(:,7));
legend('motor0','motor1');
%}

% gimbal angle
%{
ylabel('Gimbal angle (deg)');
plot(A(:,1),A(:,2));
%}









