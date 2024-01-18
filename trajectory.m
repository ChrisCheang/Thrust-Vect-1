function trajectory(thetaG, thetaR, thetaGt, thetaRt, t)

    % used to compute a smooth straight trajectory with minimum jerk from a
    % current pos to a target pos. Not really needed for a ground test with
    % smooth and continuous target input (in this case maybe use filtered
    % pos control on odrive?), but may be important for very fast target
    % position changes (e.g. joystick control or flight control algorithm)

    % Inputs: current nRotA, nRotB, nRotAdot, nRotBdot, nRotAt,
    % nRotBt, t (t from zero to t_final goes from current pos to target 
    % pos, practically t could be returned as the next timestep every time 
    % the target pos is updated) (Odrive seems to use turns as unit)

    % outputs: target position at time t (taken by evaluating the pp
    % polynomial output by minjerkpolytraj)

    % tentatively: the standard trajectory control scheme for odrive is
    % trapezoidal point to point trajectory, so might be better to use
    % filtered position control from a list of way or via points

    % main idea:

    % convert inputs (based on rotation)
    % generate time and space-scaled (normalised) minjerk trajectory
    % generate waypoints in cartesian C-space (configuration space)
    % generate minjerk trajectory in cartesian c-space
    % convert to thetaG and thetaR , then rotA and rotB waypoints OR the
    % interpolated target point for time t
    

    ga = thetaG*cos(thetaR) + t*(thetaGt*cos(thetaRt)-thetaG*cos(thetaR));
    gb = thetaG*sin(thetaR) + t*(thetaGt*sin(thetaRt)-thetaG*sin(thetaR));
    hg = sqrt(ga^2+gb^2);
    hr = atan2(gb,ga);

    wpts = [0 1];

    tpts = 0:1;
    
    numsamples = 100;
    
    [q,qd,qdd,qddd,pp,timepoints,tsamples] = minjerkpolytraj(wpts,tpts,numsamples,VelocityBoundaryCondition=[0 0]);
    
    plot(tsamples,q);

end
