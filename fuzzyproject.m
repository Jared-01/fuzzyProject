
% 
%  File    :   fuzzyProject.m  
% 
%  Author  :   Jared Murray 
%  Date    :   10/01/2021 
% 
%  Course  :   CSE 454
%   
%  Description : 
% %   This code takes as input a starting position and a target
%     position of a robot. The code acts as a robot controller
%     allowing the robot navigate to the target. It does this by 
%     using the current position and angle of the robot as inputs to 
%     a fuzzifier and a fuzzy inference engine which allows the controller 
%     to determine via a defuzzifier what force and angular velocity to
%     push the robot forward with. The controller changes the robots's 
%     forward force and angular velocity every 100ms until it get to the 
%     target location
%
%  To use this script, input the robots starting and target positions
%  using x, y coordinates.                 
% 

%Prompt user to input starting and ending x, y coordinates
x1 = input("Enter robot starting x position");
y1 = input("Enter robot starting y position");
x2 = input("Enter robot target x position");
y2 = input("Enter robot target y position");

%calculate robot's starting distance using pythagorean theorem
startDistance = sqrt((x1 - x2)^2 + (y1 - y2)^2);

%calculate the angle in degrees of the target 
%postion relative the positive direction of the x-axis
targetAngle = atan2((y2 - y1), (x2 - x1)) * (180/pi);
%convert negative angles to positive ones
if targetAngle < 0 
    targetAngle = 2 * 180 + targetAngle; 
end
%Initialize robot's movement and angle
angularVelocity = 0;
velocity = 0;
angleFacing = 0;
DistanceToTarget = startDistance;

startDistance
targetAngle

%Distance membership functions

syms d;
close(d) = -d/(startDistance/3) + 1;
%close membership function as a peicewise function
pwClose = piecewise(d < 0, 0,...      
    0 <= d < startDistance/3, close,d >= startDistance/3, 0);

mid_1(d) = d/(startDistance/4) - 1;
mid_2(d) = -d/((startDistance)/4) + 3;
%mid-distance membership function as a peicewise function
pwMid = piecewise(d < 0, 0, 0 <= d < startDistance/4, 0,... 
    startDistance/4 <= d < startDistance/2, mid_1,...
    startDistance/2 <= d < 3 * startDistance/4, mid_2,...
    d >= 3 * startDistance/4, 0);     

far(d) = d/(startDistance/3) - 2;
%far membership function as a peicewise function
pwFar = piecewise(d < 2 * startDistance/3, 0,...  
    2 * startDistance/3 <= d < startDistance, far,...
    d >= startDistance, 1);

figure (1) %figure 1 will plot close, mid, and far membership functions
fplot(pwClose, [0, startDistance]);
hold on
fplot(pwMid, [0, startDistance]);
fplot(pwFar, [0, startDistance]);
title('Distance Membership Functions')
xlabel('Distance (units)') 
ylabel('Membership') 
legend({'Close Membership',...
    'Mid-Distance Membership', 'Far Membership'})
hold off

%Velocity membership functions

%velocity has uits of (distance unit)/ms
syms v;
slow(v) = -v/(startDistance/500) + 1;
%slow membership function as a peicewise function
pwSlow = piecewise(v < 0, 0,...
    0 <= v < startDistance/500, slow, v >= startDistance/500, 0);

midV_1(v) = (2800 * v)/(startDistance * 3) - 4/3;
midV_2(v) = -(1200 * v)/startDistance + 4;
%midium velocity membership function as a peicewise function
pwMidV = piecewise(v < 0, 0, 0 <= v < startDistance/700, 0,...
    startDistance/700 <= v < startDistance/400, midV_1,...
    startDistance/400 <= v < startDistance/300, midV_2,...
    v >= startDistance/300, 0);

%fast membership function as a peicewise function
fast(v) = (1400 * v)/(startDistance * 3) - 4/3;
pwFast = piecewise(v < startDistance/350, 0,...
   startDistance/350 <= v < startDistance/200, fast,...
    v >= startDistance/200, 1);

figure (2) %figure 2 will plot close, mid, and far membership functions
fplot(pwSlow, [0, startDistance/200]);
hold on
fplot(pwMidV, [0, startDistance/200]);
fplot(pwFast, [0, startDistance/200]);
title('Velocity Membership Functions')
xlabel('Velocity (units/ms)') 
ylabel('Membership') 
legend({'Slow Membership',...
    'Mid-Velocity Membership', 'Fast Membership'})
hold off 

%Angle membership functions

syms a;
smallerAngle(a) = -(5 * a)/targetAngle + 1;
%Smaller Angle membership function as a peicewise function
pwSmallerAngle = piecewise(a < 0, 0,...
    0 <= a < targetAngle/5, smallerAngle, a >= targetAngle/5, 0);

slightlySmallerAngle1(a) = (12 * a)/(targetAngle * 7) - 2/7;
slightlySmallerAngle2(a) = -(4 * a)/(targetAngle) + 4;
%Slightly Smaller Angle membership function as a peicewise function
pwSlightlySmallerAngle = piecewise(a < 0, 0, 0 <= a < targetAngle/6, 0,...
    targetAngle/6 <= a < (3 * targetAngle)/4, slightlySmallerAngle1,...
    (3 * targetAngle)/4 <= a < targetAngle, slightlySmallerAngle2,...
    a >= targetAngle, 0);

facingTarget1(a) = (8 * a)/(targetAngle) - 7;
facingTarget2(a) = -(8 * a)/(360 - targetAngle) + 1 +...
    (8 * targetAngle)/(360 - targetAngle);
%Facing target membership function as a peicewise function
pwFacingTarget = piecewise(a < 0, 0, 0 <= a < 7 * targetAngle/8, 0,...
    7 * targetAngle/8 <= a < targetAngle, facingTarget1,...
    targetAngle <= a < targetAngle + (360 - targetAngle)/8, ...
    facingTarget2, a >= targetAngle + (360 - targetAngle)/8, 0);

slightlyLargerAngle1(a) = (4 * a)/(360 - targetAngle) - ...
    (4 * targetAngle)/(360 - targetAngle);
slightlyLargerAngle2(a) = -(12 * a)/(7 * (360 - targetAngle))+...
    (12 * targetAngle)/(7 * (360 - targetAngle))...
    + 60 * (360 - targetAngle)/(42 * (360 - targetAngle));
%Slightly larger Angle membership function as a peicewise function
pwSlightlyLargerAngle = piecewise(a < 0, 0, 0 <= a < targetAngle, 0,...
    targetAngle <= a < targetAngle + (360 - targetAngle)/4, ...
    slightlyLargerAngle1, targetAngle + (360 - targetAngle)/4 <= a < ...
    targetAngle + 5  * (360 - targetAngle)/6, slightlyLargerAngle2,...
    a >= targetAngle + 5  * (360 - targetAngle)/6, 0);

largerAngle(a) = -a/(targetAngle + 4 * (360 - targetAngle)/5 - 360)...
    + 1 + 360/(targetAngle + 4 * (360 - targetAngle)/5 - 360);
%Slightly Smaller Angle membership function as a peicewise function
pwLargerAngle = piecewise(a < targetAngle + 4 * (360 - targetAngle)/5, 0,...
    targetAngle + 4 * (360 - targetAngle)/5 <= a <= 360, largerAngle);

figure (3) %figure 3 will plot all angle membership functions
fplot(pwSmallerAngle, [0, 360]);
hold on
fplot(pwSlightlySmallerAngle, [0, 360]);
fplot(pwFacingTarget, [0, 360]);
fplot(pwSlightlyLargerAngle, [0, 360]);
fplot(pwLargerAngle, [0, 360]);
title('Angle Membership Functions')
xlabel('Angle (degrees)') 
ylabel('Membership') 
legend({'Smaller Angle Membership',...
    'Slightly Smaller Angle Membership', 'Facing Membership',...
    'Slightly Larger Angle Membership', 'Larger Angle Memebership'})
hold off 

%Force components for defuzzifier

%All force components where calculated assuming the robot weighs 1kg
%Force components have units of kg * (distance unit)/ms^2
noPush = 0;
%Positive forces are directed at the angle the robot is facing
posHard = startDistance/(80000);
posSoft = startDistance/(140000);
%Negative forces are directed at the opposite angle the robot is facing
negHard = -startDistance/(100000);
negSoft = -startDistance/(170000);

%Angular velocity components for defuzzifier

%Angular velocity components have units of degrees/ms
%Counter clockwise angular velocity increases the angle the robot is facing
CounterclkFast = 30/100;
CounterclkSlow = 20/100;
noTurn = 0;
%Clockwise angular velocity decreases the angle the robot is facing
clkFast = -30/100;
clkSlow = -20/100;

%initialize variable updates
angleFacing_new = 0;
velocity_new = 0;
DistanceToTarget_new = DistanceToTarget;
targetAngle_new = targetAngle;
time = 0;

%While loop will output new angular velocity, forward force, distance from
%target, angle to target, and membership function weights every 100ms
%until the robot's distance from the target is 1/8 the starting distance
while DistanceToTarget >= startDistance/8
   
    %update variables to their new values after 100ms
    angleFacing = angleFacing_new;
    velocity = velocity_new;
    DistanceToTarget = DistanceToTarget_new;
    targetAngle = targetAngle_new;
    
    
%The fuzzifier:

%Calculate membership matrices by using the robots current position, 
%velocity and angle data as inputs to each peicewise membership function
%and using their outputs as doubles. 

    distanceMemberships = [double(pwClose(DistanceToTarget)), ...
        double(pwMid(DistanceToTarget)), double(pwFar(DistanceToTarget))];
    velocityMemberships = [double(pwSlow(velocity)),...
        double(pwMidV(velocity)), double(pwFast(velocity))];
%The force fuzzy inference engine

%The force fuzzy inference engine calculates force weights by taking the max
%of every combination of distance and velocity membership values 

    wfastfar = max(distanceMemberships(3), velocityMemberships(3));
    wfastmid = max(distanceMemberships(2), velocityMemberships(3));
    wfastclose = max(distanceMemberships(1), velocityMemberships(3));

    wmidfar = max(distanceMemberships(3), velocityMemberships(2));
    wmidmid = max(distanceMemberships(2), velocityMemberships(2));
    wmidclose = max(distanceMemberships(1), velocityMemberships(2));

    wslowfar = max(distanceMemberships(3), velocityMemberships(1));
    wslowmid = max(distanceMemberships(2), velocityMemberships(1));
    wslowclose = max(distanceMemberships(1), velocityMemberships(1));
    
%The force defuzzifier

%The force defuzzifier multiplies force components with 
% their assigned weights determined by the fuzzy inference engine

    noPushWeights = (wfastfar + wmidmid + wslowclose) * noPush;
    posHardWeights = wslowfar * posHard;
    posSoftWeights = (wmidfar + wslowmid) * posSoft;
    negHardWeights = wfastclose * negHard;
    negSoftWeights = (wfastmid + wfastclose) * negSoft;

%The output force used to push the robot is determined by the sum of the 
%force compoenets multiplied their weights divided by the sum of all the 
%weights
    forceWeightSum = wmidfar + wfastmid + wfastclose + wmidfar + wmidmid...
        + wmidclose + wslowfar + wslowmid + wslowclose;
    outForce = (noPushWeights + posHardWeights + posSoftWeights...
        + negHardWeights + negSoftWeights)/forceWeightSum;


%Since the assumed weight of the robot is 1kg, the acceleration of the 
%robot as a result of the force applied to it is the same value as the
%force
    outAcceleration = outForce;
%The distance that the robot travels over a 100ms interval is calculated
%using the equation d = vstart(time) + 0.5(acceleration)(time^2)
    disTraveled = velocity * 100 + 0.5 * outAcceleration * 100 * 100;
%The velocity the robot is traveling at the end of 100ms is calculated
%using the equation v = vstart + acceleration * time
    velocity_new = velocity + outAcceleration * 100;
%The new x and  y coordinates the robot are at after 100ms are calculated
%to determine the distance traveled
    x1 = x1 + disTraveled * cos(angleFacing);
    y1 = y1 + disTraveled * sin(angleFacing);
    DistanceToTarget_new = sqrt((x1 - x2)^2 + (y1 - y2)^2);
%The new angle between the positive x-axis and the target position is
%calculated the end of the 100ms. This angle is updated becuase it 
%is calculated relative to the robot's position. 
    targetAngle_new = atan2((y2 - y1), (x2 - x1)) * (180/pi);
    if targetAngle_new < 0 
         targetAngle_new = 2 * 180 + targetAngle_new;
    end
    
%The robot will predict what the target angle (angle between target and 
%positive x-axis )will be at the end of the current 100ms period based on
%the current angle it's facing and the applied force. This preicted taret
%angle will be used to generate new angle membership functions. 
    smallerAngle_new(a) = -(5 * a)/targetAngle_new + 1;
    pwSmallerAngle_new = piecewise(a < 0, 0,...
        0 <= a < targetAngle_new/5, smallerAngle_new, a >= ...
        targetAngle_new/5, 0);

    slightlySmallerAngle1_new(a) = (12 * a)/(targetAngle_new * 7) - 2/7;
    slightlySmallerAngle2_new(a) = -(4 * a)/(targetAngle_new) + 4;
    pwSlightlySmallerAngle_new = piecewise(a < 0, 0, 0 <= a <...
        targetAngle_new/6, 0, targetAngle_new/6 <= a <...
        (3 * targetAngle_new)/4, slightlySmallerAngle1_new,...
        (3 * targetAngle_new)/4 <= a < targetAngle_new, ...
        slightlySmallerAngle2_new,a >= targetAngle_new, 0);

    facingTarget1_new(a) = (8 * a)/(targetAngle_new) - 7;
    facingTarget2_new(a) = -(8 * a)/(360 - targetAngle_new) + 1 + ...
        (8 * targetAngle_new)/(360 - targetAngle_new);
    pwFacingTarget_new= piecewise(a < 0, 0, 0 <= a < 7 * targetAngle_new/8, 0,...
        7 * targetAngle_new/8 <= a < targetAngle_new, facingTarget1_new,...
        targetAngle_new <= a < targetAngle_new + ...
        (360 - targetAngle_new)/8, facingTarget2_new,...
        a >= targetAngle_new + (360 - targetAngle_new)/8, 0);

    slightlyLargerAngle1_new(a) = (4 * a)/(360 - targetAngle_new) -...
        (4 * targetAngle_new)/(360 - targetAngle_new);
    slightlyLargerAngle2_new(a) = -(12 * a)/(7 * (360 - targetAngle_new))+...
        (12 * targetAngle_new)/(7 * (360 - targetAngle_new))...
        + 60 * (360 - targetAngle_new)/(42 * (360 - targetAngle_new));
    pwSlightlyLargerAngle_new= piecewise(a < 0, 0, 0 <= a < targetAngle_new, 0,...
        targetAngle_new <= a < targetAngle_new + (360 - targetAngle_new)/4, ...
        slightlyLargerAngle1_new, targetAngle_new + (360 - targetAngle_new)/4 <= a...
        < targetAngle_new + 5  * (360 - targetAngle_new)/6, slightlyLargerAngle2_new,...
        a >= targetAngle_new + 5  * (360 - targetAngle_new)/6, 0);

    largerAngle_new(a) = -a/(targetAngle_new + 4 * ...
        (360 - targetAngle_new)/5 - 360)+ 1 + 360/(targetAngle_new + ...
        4 * (360 - targetAngle_new)/5 - 360);
    pwLargerAngle_new= piecewise(a <= targetAngle_new + 4 * ...
        (360 - targetAngle_new)/5, 0, targetAngle_new + 4 * ...
        (360 - targetAngle_new)/5 < a <= 360, largerAngle_new);
    
    %Angle Fuzzifier:
    
    %The angle fuzzifier creates a matrix of weights calculated using the
    %the current angle the robot is facing as input to all of the
    %angle membership funcions
     AngleWeights = [double(pwSmallerAngle_new(angleFacing)), ...
         double(pwSlightlySmallerAngle_new(angleFacing)), ...
         double(pwFacingTarget_new(angleFacing)),...
         double(pwLargerAngle_new(angleFacing)),...
         double(pwSlightlyLargerAngle_new(angleFacing))];
    
    
    %Angle Fuzzy Inference Engine :
    
    %The angle fuzzy inference engine multiplies each angular velocity
    %component with its correspoding weight, sums the resuls, then divides
    %the sum by the sum of the weights to give the output angular velocity
    CounterclkFast_W = AngleWeights(1) * CounterclkFast;
    CounterclkSlow_W = AngleWeights(2) * CounterclkSlow;
    noTurn_W = AngleWeights(3) * noTurn;
    clkFast_W = AngleWeights(4) * clkFast;
    clkSlow_W = AngleWeights(5) * clkSlow;

    angleWeightSum = AngleWeights(1) + AngleWeights(2)...
        + AngleWeights(3) + AngleWeights(4) + AngleWeights(5);
    outAngVelocity = (CounterclkFast_W + CounterclkSlow_W + noTurn_W...
        + clkFast_W + clkSlow_W)/angleWeightSum;
    
    %Calculation for angle robot is facing at end of 100ms
    angleFacing_new = angleFacing + outAngVelocity * 100;
    
    %output update robot info every 100ms
    time
    distanceMemberships
    velocityMemberships
    AngleWeights
    outForce = double(outForce)
    outAngVelocity = double(outAngVelocity)
    DistanceToTarget = double(DistanceToTarget)
    angleOffset = targetAngle - double(angleFacing)
    
    %time is in ms 
    time = time + 100;

end

