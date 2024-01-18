function thetas = tvcForward(nRotA,nRotB,rEngine,lPivot,rMount,hMount)

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
    function lengths = actuator_lengths_from_cartesian(A,B,rEngine,lPivot,rMount,hMount)
    
        act1MountEngine = [rEngine,0,-lPivot];
        act1MountEngine = gimbal_cartesian(act1MountEngine,A,B);
        length1 = distance([rMount,0,hMount],act1MountEngine);
    
        act2MountEngine = [0,rEngine,-lPivot];
        act2MountEngine = gimbal_cartesian(act2MountEngine,A,B);
        length2 = distance([0,rMount,hMount],act2MountEngine);

        lengths = [length1, length2];
    end
    %
    
    %
    function limits = actuator_limits(rEngine,lPivot,rMount,hMount,aMax)
        %Consider the limits of actuator B as thetaA goes from -aMax to aMax
        BFullRetract = actuator_lengths_from_cartesian(-aMax,0,rEngine,lPivot,rMount,hMount);
        BFullExtend = actuator_lengths_from_cartesian(aMax,0,rEngine,lPivot,rMount,hMount);
        min = BFullRetract(2);
        max = BFullExtend(2);
        limits = [min, max];
    end
    %
    
    function length = actuator_neutral(rEngine,lPivot,rMount,hMount)
        lengths = actuator_lengths_from_cartesian(0,0,rEngine,lPivot,rMount,hMount);
        length = lengths(1);
    end
    
    %
    function thetas = cartesian_from_actuator_lengths(actA,actB,rEngine,lPivot,rMount,hMount,aMax)
        function thetas = cartesian_from_actuator_lengths_estimate(actA,actB,rEngine,lPivot,rMount,hMount,aMax)
        
            %Uses and iterative method, based on a linear actuator length - gimbal
            %angle assumption for first guess
            limits = actuator_limits(rEngine,lPivot,rMount,hMount,aMax);
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
            thetas = [thetaA, thetaB];
        end
        function F = root2d(x);  %x1 = thetaA, x2 = thetaB
            F = actuator_lengths_from_cartesian(x(1),x(2),rEngine,lPivot,rMount,hMount) - [actA,actB];
        end
        fun = @root2d;
        options = optimoptions('fsolve','Display','Off');
        x0 = cartesian_from_actuator_lengths_estimate(actA,actB,rEngine,lPivot,rMount,hMount,aMax);
        x = fsolve(fun,x0,options);
        
        thetaALocal = x(1);
        thetaBLocal = x(2);
        thetas = [thetaALocal, thetaBLocal];
    end
    %

    function thetas = cartesian_to_polar(A, B)
        % Gimbal angle (polarAngles(1)) range = 0 < theta < pi/2, Roll angle
        % (polarAngles(2)) range = -pi < 0 < pi, 0 is x-axis
        point = gimbal_cartesian([0,0,-1],A,B);
        thetaGLocal = atan2(norm(cross(point,[0,0,-1])), dot(point,[0,0,-1]));
        thetaRLocal = atan2(point(2),point(1));
        thetas = [thetaGLocal, thetaRLocal];
    end
    
    function thetas = unit_point_to_cartesian(point)
        thetaGLocal = atan2(norm(cross(point,[0,0,-1])), dot(point,[0,0,-1]));
        thetaRLocal = atan2(point(2),point(1));
        cartesianAngles = polar_to_cartesian(thetaGLocal, thetaRLocal);
        thetaA = cartesianAngles(1);
        thetaB = cartesianAngles(2);
        thetas = [thetaA, thetaB];
    end
    
    function thetas = polar_to_cartesian(thetaG, thetaR)
        axes = [1,0,0];
        axes = rotate(axes,thetaR-pi/2,[0,0,0],[0,0,1]); % use this to define the axis of rotation for polar gimbal angle
        point = [0,0,-1];
        point = rotate(point,thetaG,[0,0,0],axes);
       
        N = [1,0,0];
        thetaB = pi/2 - atan2(norm(cross(point,N)), dot(point,N));
    
        pointProj = [0,point(2),point(3)];
        sign = - abs(thetaR)/thetaR;
        thetaA = sign * atan2(norm(cross(pointProj,[0,0,-1])), dot(pointProj,[0,0,-1]));
        thetas = [thetaA, thetaB];
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
        lead = 4;
        delta_dis = actuator_dis-neutral_dis;
        nRotation = delta_dis/lead;
    end
    
    function nRotations = actuator_revs_from_polar(thetaG,thetaR,rEngine,lPivot,rMount,hMount)
        neutral_dis = actuator_neutral(rEngine,lPivot,rMount,hMount);
        cartesianAngles = polar_to_cartesian(thetaG, thetaR);
        A = cartesianAngles(1);
        B = cartesianAngles(2);
    
        ActLens = actuator_lengths_from_cartesian(A,B,rEngine,lPivot,rMount,hMount);
        ActALen = ActLens(1);
        ActBLen = ActLens(2);
    
        nRotation1 = MotorActuatorRevolution(neutral_dis,ActALen);
        nRotation2 = MotorActuatorRevolution(neutral_dis,ActBLen);
        nRotations = [nRotation1, nRotation2];
    end


    neutral = actuator_neutral(rEngine,lPivot,rMount,hMount);

    % Quick fix - these two values is the same with the global version!
    lead = 4;
    aMax = 10 * pi/180;


    actA = neutral + lead * nRotA;
    actB = neutral + lead * nRotB;

    thetasCartesian = cartesian_from_actuator_lengths(actA,actB,rEngine,lPivot,rMount,hMount,aMax);
    thetas = cartesian_to_polar(thetasCartesian(1),thetasCartesian(2));

end
