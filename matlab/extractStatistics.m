%% Now its a script, not a function!

thresholdArray = 0.1:0.1:3;
bag = rosbag('');

GTpersonX = 3; % Define this thing <-------------------
GTpersonY = 0; % Define this thing <-------------------

%Filter stuff %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
N = 4;                 % Order of polynomial fit
F = 41;                % Window length
HalfWin  = ((F+1)/2) -1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


positionTopic = select(bag, 'Topic', '/detectionPoints');
jointStatesTopic = select(bag, 'Topic', '/syncJointStates');

%Get the messages
positionMessages = readMessages(positionTopic);
jointMessages = readMessages(jointStatesTopic);

%Get times vectors

lastTimePos = size(positionMessages);
lastTimePos = lastTimePos(1);

timesPosition = zeros(1, lastTimePos);

lastTimeJoints = size(jointMessages);
lastTimeJoints = lastTimeJoints(1);

timesJoints = zeros(1, lastTimeJoints);

if timesPosition(1) > timesJoints(1)
   firstTime =  timesJoints(1);
else
   firstTime = timesPosition(1);
end

for i=(F+1)/2:lastTimePos-(F+1)/2
    secs = (positionMessages{i,1}.Header.Stamp.Sec - firstTime.Sec);
    nsecs = (double(positionMessages{i,1}.Header.Stamp.Nsec)*10^-9 - double(firstTime.Nsec)*10^-9);
    timesPosition(i) = double(secs)+double(nsecs);
end

for i=(F+1)/2:lastTimeJoints-(F+1)/2
    secs = (jointMessages{i,1}.Header.Stamp.Sec - firstTime.Sec);
    nsecs = (double(jointMessages{i,1}.Header.Stamp.Nsec)*10^-9 - double(firstTime.Nsec)*10^-9);
    timesJoints(i) = double(secs)+double(nsecs);
end

%% Make the data arrays
% Neck position

jointNeckPosition = zeros(1, lastTimeJoints);
for i=1:lastTimeJoints
jointNeckPosition(i) = jointMessages{i,1}.Position(8,1);
end

%% Left eye position
% Vergence
jointEyeVergencePosition = zeros(1, lastTimeJoints);
for i=1:lastTimeJoints
jointEyeVergencePosition(i) = -jointMessages{i,1}.Position(16,1);
end

%Version
jointEyeVersionPosition = zeros(1, lastTimeJoints);
for i=1:lastTimeJoints
jointEyeVersionPosition(i) = -jointMessages{i,1}.Position(12,1);
end

%Person positions
personX = zeros(1, lastTimePos);
personY = zeros(1, lastTimePos);

for i=1:lastTimePos
    personX(i) = positionMessages{i, 1}.Point.X;
    personY(i) = positionMessages{i, 1}.Point.Y;
end

personX = personX((F+1)/2:lastTimeJoints-(F+1)/2);
personY = personY((F+1)/2:lastTimeJoints-(F+1)/2);


%% Compute the velocities from the jointPositions using a Savitzky-Golay filter

% the differential
dt = timesJoints((F+1)/2+1) - timesJoints((F+1)/2);

% velocities
jointNeckVelocity = computeVelocities(jointNeckPosition, N, F, HalfWin, dt);

jointEyeVersionVelocity = computeVelocities(jointEyeVersionPosition, N, F, HalfWin, dt);
jointEyeVergenceVelocity = computeVelocitites(jointEyeVergencePosition, N, F, HalfWin, dt);


%% Fuse velocities
leftEyeVelocity = jointEyeVersionVelocity+jointEyeVergenceVelocity/2;
overallVelocity = leftEyeVelocity + jointNeckVelocity;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
figure(1);
hold on
plot(times((F+1)/2:lastTimeJoints-(F+1)/2), leftEyeVelocity*180/pi());
plot(times((F+1)/2:lastTimeJoints-(F+1)/2), jointNeckVelocity*180/pi());
plot(times((F+1)/2:lastTimeJoints-(F+1)/2), overallVelocity*180/pi());
title('Velocity plots');
xlabel('Time (s)');
ylabel('Angular velocity (degrees/s)');
legend('Left eye velocity', 'Neck velocity', 'Summed velocity');
hold off


%Plot showing the threshold

figure(2)
hold on
plot(times((F+1)/2:lastTimeJoints-(F+1)/2), leftEyeVelocity*180/pi());
plot(times((F+1)/2:lastTimeJoints-(F+1)/2), jointNeckVelocity*180/pi());

sizethresh = size(times((F+1)/2:lastTimeJoints-(F+1)/2));
sizethresh = sizethresh(1);

thresh = ones(1, sizethresh)*thresholdArray(end/2)*180/pi(); %An exemple of one

plot(times((F+1)/2:lastTimeJoints-(F+1)/2), thresh);

title('Discarded information - Example threshold');
xlabel('Time (s)');
ylabel('Angular velocity (degrees/s)');
legend('Left eye velocity', 'Neck velocity', 'Threshold');
hold off



%% Finally... compute the statistics
sz = size(leftEyeVelocity);
sz = sz(1);

numOfThreshs = size(thresholdArray);
numOfThreshs = numOfThreshs(1);

l = 1;

for i=1:numOfThreshs
    
    thr = thresholdArray(i);

    b = 1;
    for n=1:sz
        if leftEyeVelocity < thr && jointNeckVelocity < thr
            posXunderVth(b) = personX(n);
            posYunderVth(b) = personY(n);
            b = b+1;
        end 
    end
    
    
    
    testSz = size(posXunderVth);
    testSz = testSz(1);
    testSz2 = size(posYunderVth);
    testSz2 = testSz2(1);
    
    
    if testSz ~= testSz2
       disp('Warning: posXunderVth and posYunderVth dont have the same size! The code has bugs!');
    end

    if testSz > 0 && testSz2 > 0
        
        if testSz < 10 
            disp(['Warning: less than 10 samples gathered for velocity threshold: ' num2str(thr)]);
        end
        
        meanArrayX(l) = mean(posXunderVth-GT);
        meanArrayY(l) = mean(posYunderVth-GT);
        varianceArrayX(l) = var(posXunderVth-GT);
        varianceArrayY(l) = var(posYunderVth-GT);
        thresholdArrayOut(l) = thr;
        l = l + 1;
    end
end


%% And plot the statistics

figure(3)
hold on
plot(thresholdArrayOut*180/pi(), meanArrayX);
plot(thresholdArrayOut*180/pi(), meanArrayY);
title('Position error mean for X and Y coordinates');
xlabel('Angular Velocity Threshold (degrees/s)');
ylabel('Mean error (m)');
legend('Mean error X', 'Mean error Y');
hold off

figure(4)
hold on
plot(thresholdArrayOut*180/pi(), varianceArrayX);
plot(thresholdArrayOut*180/pi(), varianceArrayY);
title('Position error variance for X and Y coordinates');
xlabel('Angular Velocity Threshold (degrees/s)');
ylabel('Variance of error (m)');
legend('Variance of X error', 'Variance of Y error');
hold off

