%% Now its a script, not a function!
close all
clear

FONTSIZE_ = 45;

thresholdArray = 0:0.1:600*pi()/180;
bag = rosbag('/home/avelino/catkin_ws/src/bio_visual_suppression/bio_visual_suppression/bags/gaze_bag_out.bag');

GTpersonX = 3.1; % Define this thing <-------------------
GTpersonY = 0; % Define this thing <-------------------

%Filter stuff %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
N = 2;                 % Order of polynomial fit
F = 7;                % Window length
HalfWin  = ((F+1)/2) -1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


positionTopic = select(bag, 'Topic', '/detectionPoints');
jointStatesTopic = select(bag, 'Topic', '/syncJointStates');
feetTopic = select(bag, 'Topic', '/feetInImage');

%Get the messages
positionMessages = readMessages(positionTopic);
jointMessages = readMessages(jointStatesTopic);
feetMessages = readMessages(feetTopic);

%Get times vectors

lastTimePos = size(positionMessages);
lastTimePos = lastTimePos(1);

timesPosition = zeros(1, lastTimePos);

lastTimeJoints = size(jointMessages);
lastTimeJoints = lastTimeJoints(1);

timesJoints = zeros(1, lastTimeJoints);

firstPoistionTime = double(positionMessages{1,1}.Header.Stamp.Sec) + double(double(positionMessages{1,1}.Header.Stamp.Nsec)*10^-9);
firstJointTime = double(jointMessages{1,1}.Header.Stamp.Sec) + double(double(jointMessages{1,1}.Header.Stamp.Nsec)*10^-9);

if firstPoistionTime > firstJointTime
   firstTime =  jointMessages{1,1}.Header.Stamp;
else
   firstTime = positionMessages{1,1}.Header.Stamp;
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
jointLeftEye = zeros(1, lastTimeJoints);
for i=1:lastTimeJoints
jointLeftEye(i) = -jointMessages{i,1}.Position(11,1);
end

%% Right eye position
jointRightEye = zeros(1, lastTimeJoints);
for i=1:lastTimeJoints
jointRightEye(i) = -jointMessages{i,1}.Position(12,1);
end

%Person positions
personX = zeros(1, lastTimePos);
personY = zeros(1, lastTimePos);

imageX = zeros(1, lastTimePos);
imageY = zeros(1, lastTimePos);

for i=1:lastTimePos
    personX(i) = positionMessages{i, 1}.Point.X;
    personY(i) = positionMessages{i, 1}.Point.Y;
    imageX(i) = feetMessages{i, 1}.Point.X;
    imageY(i) = feetMessages{i, 1}.Point.Y;
end

personX = personX((F+1)/2:lastTimeJoints-(F+1)/2);
personY = personY((F+1)/2:lastTimeJoints-(F+1)/2);

imageX = imageX((F+1)/2:lastTimeJoints-(F+1)/2);
imageY = imageY((F+1)/2:lastTimeJoints-(F+1)/2);


%% Compute the velocities from the jointPositions using a Savitzky-Golay filter

% the differential

times_aux = timesJoints((F+1)/2:lastTimePos-(F+1)/2);
dt = mean(diff(times_aux));

% velocities
jointNeckVelocity = computeVelocities(jointNeckPosition, N, F, HalfWin, dt, lastTimeJoints);

rightEyeVelocity = computeVelocities(jointRightEye, N, F, HalfWin, dt, lastTimeJoints);
leftEyeVelocity = computeVelocities(jointLeftEye, N, F, HalfWin, dt, lastTimeJoints);


%% Fuse velocities
% leftEyeVelocity = jointEyeVersionVelocity+jointEyeVergenceVelocity/2;
overallVelocity = leftEyeVelocity + jointNeckVelocity;
overallPosition = jointLeftEye+jointNeckPosition;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%55
figure(1);

a = axes;

set(a,'TickLabelInterpreter', 'latex');

set(gca,'fontsize',FONTSIZE_);

hold on
plot(timesJoints((F+1)/2:lastTimeJoints-(F+1)/2), leftEyeVelocity*180/pi(), 'LineStyle', ':', 'LineWidth',5);
plot(timesJoints((F+1)/2:lastTimeJoints-(F+1)/2), jointNeckVelocity*180/pi(), 'LineStyle', ':', 'LineWidth',5);
plot(timesJoints((F+1)/2:lastTimeJoints-(F+1)/2), overallVelocity*180/pi(), 'r', 'LineWidth',5);
%title('Joints angular velocity profile');
xlabel('Time (s)');
ylabel('Angular velocity (degrees/s)');
legend('Left eye velocity', 'Neck velocity', 'Combined velocity');
hold off

set(gca,'XTick', [18 19 20 21 22] );

xlim([18.53 22]);
ylim([-600, 600]);

set(gcf,'color','w');


%Plot showing the threshold

% figure(2)
% set(gca,'fontsize',FONTSIZE_);
% hold on
% plot(timesJoints((F+1)/2:lastTimeJoints-(F+1)/2), leftEyeVelocity*180/pi());
% 
% sizethresh = size(timesJoints((F+1)/2:lastTimeJoints-(F+1)/2));
% sizethresh = sizethresh(2);
% 
% thresh = ones(1, sizethresh)*thresholdArray(end/2)*180/pi(); %An exemple of one
% neg_thresh = -thresh;
% 
% 
% plot(timesJoints((F+1)/2:lastTimeJoints-(F+1)/2), thresh);
% 
% plot(timesJoints((F+1)/2:lastTimeJoints-(F+1)/2) ,imageX);
% 
% plot(timesJoints((F+1)/2:lastTimeJoints-(F+1)/2), neg_thresh);
% 
% title('Example thresholds for visual suppression');
% xlabel('Time (s)');
% ylabel('Angular velocity (degrees/s)');
% legend('Combined velocity', 'Positive threshold', 'Negative threshold');
% hold off
% 
% set(gcf,'color','w');
% 
% xlim([18.53 22]);
% ylim([-600, 600]);


%%
figure(3)
a = axes;

set(a,'TickLabelInterpreter', 'latex');

set(gca,'fontsize',FONTSIZE_);

hold on


aux = abs(overallVelocity);
auxTimes = timesJoints((F+1)/2:lastTimeJoints-(F+1)/2);

num = numel(auxTimes); 
xi = interp1( 1:num, auxTimes, linspace(1, num, 500*num) );
yi = interp1(auxTimes, aux, xi );

sizethresh = size(xi);
sizethresh = sizethresh(2);

thresh = ones(1, sizethresh)*150*pi()/180; %An exemple of one


nonSuppressionRegion = yi(yi<thresh);
timesNonSuppression = xi(yi<thresh);

h1 = plot(timesNonSuppression, nonSuppressionRegion*180/pi(), 'LineWidth',7);

plot(xi, thresh*180/pi(), 'white', 'LineWidth',10);

h2 = plot(timesJoints((F+1)/2:lastTimeJoints-(F+1)/2), aux*180/pi(), 'Color', 'red' ,'LineStyle', '-.', 'LineWidth',5);

h3 = plot(xi, thresh*180/pi(), 'LineWidth',5);



%title('Example threshold for visual suppression');
xlabel('Time (s)');
ylabel('Angular velocity (degrees/s)');
legend([h1, h2, h3], {'Non-Suppression velocity', 'Absolute combined velocity', 'Suppression Threshold'});
hold off


xlim([18.53 22]);
ylim([0, 600]);

set(gca,'YTick', [150 300 600] );

set(gca,'XTick', [18 19 20 21 22] );


set(gca,'YTickLabel', {'$\omega_{th}$', 300 , 600});

set(gcf,'color','w');


%% Finally... compute the statistics
sz = size(leftEyeVelocity);
sz = sz(2);

numOfThreshs = size(thresholdArray);
numOfThreshs = numOfThreshs(2);
distanceError = [];

l = 1;

for i=1:numOfThreshs-1
    
    thr = thresholdArray(i);
    thr1 = thresholdArray(i+1);
    clear posXunderVth;
    clear posYunderVth;
    posXunderVth = [];
    posYunderVth = [];
    b = 1;
    for n=1:sz
        if (abs(leftEyeVelocity(n)+jointNeckVelocity(n)) < thr1 && abs(leftEyeVelocity(n)+jointNeckVelocity(n)) > thr)
            posXunderVth(b) = personX(n);
            posYunderVth(b) = personY(n);
            imageXunderVth(b) = imageX(n);
            imageYunderVth(b) = imageY(n);
            timesAfterSupr(b) = times_aux(n);
            b = b+1;
        end
    end
    
    timesSup{i} = timesAfterSupr;
    imageYSup{i} = imageYunderVth;
    imageXSup{i} = imageXunderVth;
    posYsup{i} = posYunderVth;
    
    testSz = size(posXunderVth);
    testSz = testSz(2);
    testSz2 = size(posYunderVth);
    testSz2 = testSz2(2);
    
    
    if testSz ~= testSz2
       disp('Warning: posXunderVth and posYunderVth dont have the same size! The code has bugs!');
    end

    if testSz > 0 && testSz2 > 0
        
        if testSz < 10 
            disp(['Warning: less than 10 samples gathered for velocity threshold: ' num2str(thr)]);
        end
        
%        figure(10+i);
%         scatter(posYunderVth, posXunderVth);
%         title(['Position estimates for v_{th}= ' num2str(thresholdArray(i)*180/pi())]);
%         xlabel('Y position');
%         ylabel('X position (m)');
%         legend('Estimated position');       
% 
%          plot( timesAfterSupr ,posYunderVth);
%          hold on;
%          plot(timesAfterSupr, imageXunderVth);
%          
%          h2 = plot(timesJoints((F+1)/2:lastTimeJoints-(F+1)/2), aux*180/pi()/800, 'Color', 'red' ,'LineStyle', '--', 'LineWidth',1);
%          hold off
%         
        %meanArrayX(l) = mean(posXunderVth-GTpersonX);
        %meanArrayY(l) = mean(posYunderVth-GTpersonY);
       
        
        errorX = posXunderVth-GTpersonX;
        errorY = posYunderVth-GTpersonY;

        meanArrayX(l) = mean(abs(errorX));
        meanArrayY(l) = mean(abs(errorY));
        
        normError = sqrt(errorX.^2+errorY.^2);
        
        distanceError = [distanceError mean(normError)];
        
        varianceArrayX(l) = std(posXunderVth-GTpersonX);
        varianceArrayY(l) = std(posYunderVth-GTpersonY);
        thresholdArrayOut(l) = thr;
        l = l + 1;
    end
end


%% Plot the number of samples for each bin
figure(50)
hold on
bins = thresholdArray(1:end-1)+diff(thresholdArray)/2;
bins = bins*180/pi();
nsamples = cellfun('length', posYsup);
bar(bins, nsamples);
set(gca,'XTick', thresholdArray*180/pi());
hold off

%% Plot person position and joint position to estimate the delay

% figure(20)
% hold on
% plot(timesJoints(1:end-50), (-overallPosition(1:end-50)-mean(-overallPosition(1:end-50)))*0.8);
% plot(timesSup{105}, medfilt1(imageXSup{105}-mean(imageXSup{105}), 1)/(mean(imageXSup{105})));
% hold off

%% And plot the statistics

figure(4)
b = axes;
set(b,'TickLabelInterpreter', 'latex');
set(gca,'fontsize',FONTSIZE_);


hold on

errorbar(bins(1:size(meanArrayY, 2)), abs(meanArrayY), abs(varianceArrayY), 'rx');


plot(thresholdArrayOut*180/pi(), abs(meanArrayX), 'LineWidth',5);
plot(thresholdArrayOut*180/pi(), abs(meanArrayY), 'LineWidth',5);


%title('Position error mean for X and Y coordinates');
xlabel('Angular Velocity Threshold (degrees/s)');
ylabel('Mean Absolute Error (m)');
legend('MAE_x', 'MAE_y');
%set(gca,'XTick', [30 40 60 80 100 120] );
hold off

set(gcf,'color','w');

figure(5)

bar(bins(1:size(meanArrayY, 2)), abs(varianceArrayY));

c = axes;
set(c,'TickLabelInterpreter', 'latex');
set(gca,'fontsize',FONTSIZE_);
hold on
plot(thresholdArrayOut*180/pi(), varianceArrayX-varianceArrayX(1), 'LineWidth', 5);
plot(thresholdArrayOut*180/pi(), varianceArrayY-varianceArrayY(1),  'LineWidth',5);
%title('Position error standard deviation for X and Y coordinates');
xlabel('Angular Velocity Threshold (degrees/s)');
ylabel({'Standard deviation of error (m)'}, 'Interpreter', 'latex');
legend('\sigma_x', '\sigma_y');
hold off

set(gcf,'color','w');

%% Plot person position along time for various thresholds

figure(5)

d = axes;
set(d,'TickLabelInterpreter', 'latex');
set(gca,'fontsize',FONTSIZE_);
hold on

gtYplot = ones(1, 50)*0.103;

plot(timesSup{1}, posYsup{1}, 'LineWidth', 3);
plot(timesSup{105}, posYsup{105}, 'LineStyle', ':', 'LineWidth', 5);
plot(gtYplot,'LineStyle', '--', 'LineWidth', 5);


xlabel('Time (s)');
ylabel({'Y person position (m)'}, 'Interpreter', 'latex');
legend('\omega_{th} = 23.5 (º/s)', '\omega_{th} = \infty', 'Person GT');
hold off
xlim([12.09 17]);
set(gca,'XTick', [18 19 20 21 22] );

set(gcf,'color','w');