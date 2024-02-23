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

%read csv (import data first)
D = testhotfiredata2302;
M = table2array(D);

%divide shadowcount by 8192 
resolution = 8192;
M(:,2) = M(:,2)./resolution;
M(:,3) = M(:,3)./resolution;

disp(M(:,2:5));

%A = tvcForward(M(,M(:,3),rEngine,lPivot,rMount,hMount)

A = ones(length(M(:,2)),4);

for t = 1:length(M(:,1))
    thetas = tvcForward(M(t,2),M(t,3),rEngine,lPivot,rMount,hMount);
    A(t,1) = 180*thetas(1)/pi; % motor 0
    A(t,2) = 180*thetas(2)/pi; % motor 1
end

clc
disp(A)






