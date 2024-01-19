function trajectory = trajectory(thetaG, thetaR, thetaGt, thetaRt, t, maxAccel)

    % used to compute a smooth straight trajectory with minimum jerk from a
    % current pos to a target pos. Not really needed for a ground test with
    % smooth and continuous target input (in this case maybe use filtered
    % pos control on odrive?), but may be important for very fast target
    % position changes (e.g. joystick control or flight control algorithm)

    % Inputs: current thetaG, thetaR, thetaGt, thetaRt, t 
    % (t from zero to t_final goes from current pos to target 
    % pos, practically t could be returned as the next timestep every time 
    % the target pos is updated) (Odrive seems to use turns as unit),
    % maxAccel (maximum angular accel in rad/s^2, will be converted to
    % turns/s^2)

    % important! for some reason, possibly from divide by zero errors, none
    % of the angles can be exactly zero.

    % outputs: target position at time t (taken by evaluating the pp
    % polynomial output by minjerkpolytraj)

    % tentatively: the standard trajectory control scheme for odrive is
    % trapezoidal point to point trajectory, so might be better to use
    % filtered position control from a list of way or via points



    % main idea:

    % convert inputs (polar angles) to cartesian C-space

    % generate time and space-scaled (normalised) minjerk trajectory
    % (same for all trajectories as start vel is ignored for now)

    % calculate start and end rotations, then scale max(qdd) to maxAccel to
    % obtain minimum finishing time and scaled time points

    % generate waypoints in cartesian C-space (configuration space)

    % generate minjerk trajectory in cartesian c-space

    % convert to thetaG and thetaR , then rotA and rotB waypoints 

    % (see if useful?) fit polynomial to rotA and rotB waypoints

   

    rEngine = 90.168;  % radius of the actuator engine mounts
    hTopRing = 55; % axial (z) distance downwards between the pivot point and the engine top ring (bottom edge)
    hEngine = 298; % axial (z) distance downwards between the pivot point and the engine bottom
    lPivot = hEngine; % axial (z) distance downwards between the pivot point and the engine actuator mount points
    hMount = 65; % axial (z) distance upwards between the pivot point and the stationary actuator mount points
    rMount = 180; % radius of the stationary actuator mounts, r=120
    aMax = 10*pi/180; % maximum gimbal angle in radians
    lead = 4; % lead of ball screw in mm
    

    % initial cartesian position


    posx1 = thetaG * cos(thetaR);
    posy1 = thetaG * sin(thetaR);
    posx2 = thetaGt * cos(thetaRt);
    posy2 = thetaGt * sin(thetaRt);

    
    % time and space scaled minjerk trajectory 
    % note: initial velocties ignored for now
    
    wpts = [0 1];
    tpts = 0:1;
    numsamples = 50;
    
    [q,qd,qdd,qddd,pp,timepoints,tsamples] = minjerkpolytraj(wpts,tpts,numsamples); 
    

    % find initial and final rotations, evalute the bigger range and use to
    % obtain scale factor for minjerk trajectory


    nRots = tvcInverse(thetaG,thetaR,rEngine,lPivot,rMount,hMount);
    nRotst = tvcInverse(thetaGt,thetaRt,rEngine,lPivot,rMount,hMount);
    rangeA = nRotst(1) - nRots(1);     % number of rotations needed to get from current pos to target pos for actuator A and B
    rangeB = nRotst(2) - nRots(2);
    maxRange = max([rangeA, rangeB]);

    % maxAccel = max(qdd)*maxRange/tf --> linearly scaling max(qdd) of the
    % normalised minjerk traj to match the given maxAccel to find tf
    scaleFactor = maxRange / maxAccel;
    tf = max(qdd) * scaleFactor;


    % generate waypoints in cartesian c-space

    % time samples for all waypoints
    tPoints = tsamples * tf;

    ga = posx1 + q*(posx2-posx1);
    gb = posy1 + q*(posy2-posy1);


    % convert to thetaG and thetaR , then rotA and rotB waypoints 


    % polar angle waypoints
    hG = sqrt(ga.^2 + gb.^2);
    hR = atan2(gb,ga);

    %plot(tPoints,hG)
    %hold on
    %plot(tPoints,hR)
    %hold off

    % rotation waypoints
    nRotAPoints = ones();
    nRotBPoints = ones();

    for i = 1:numsamples
        nRotsPoints = tvcInverse(hG(i),hR(i),rEngine,lPivot,rMount,hMount);
        nRotAPoints(i) = nRotsPoints(1);
        nRotBPoints(i) = nRotsPoints(2);
    end

    
    % graphs to compare between correct and not corrected trajectories
    %{
    plot(tPoints,nRotAPoints)
    hold on
    plot(tPoints,nRotBPoints)
    plot(tPoints, nRots(1) + q*(nRotst(1) - nRots(1)))
    plot(tPoints, nRots(2) + q*(nRotst(2) - nRots(2)))
    legend('With Correction for actuator A','With Correction for actuator B', 'Without correction for A','Without correction for B');
    hold off
    %}


    % polynomial approximations
    order = 10;
    pA = polyfit(tPoints, nRotAPoints, order);
    pB = polyfit(tPoints, nRotBPoints, order);


    % graphs to compare actual trace to order 10 polynomial approximations
    %{
    plot(tPoints,nRotAPoints)
    hold on
    plot(tPoints,nRotBPoints)
    plot(tPoints, polyval(pA,tPoints))
    plot(tPoints, polyval(pB,tPoints))
    legend('With Correction for actuator A','With Correction for actuator B', 'polyfit for A','polyfit for B');
    hold off
    %}


    % output mode 1: 3x50 array
    % trajectory = [tPoints; nRotAPoints; nRotBPoints];
    
    % output mode 2: target points at time t (t = 0 at current pos)
    nRotA = polyval(pA,min([t,tf]));
    nRotB = polyval(pB,min([t,tf]));

    trajectory = [nRotA, nRotB];


end
