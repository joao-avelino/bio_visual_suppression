function [ times neck_pan_velocity  l_eye_version_velocity l_eye_vergene_velocity] = plotNeckEyesVels( bagObject )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here


%Select the joint states topic

bagSelection = select(bagObject, 'Topic', '/joint_states');


%%%Filtro
N = 2;                 % Order of polynomial fit
F = 7;                % Window length
[b,g] = sgolay(N,F);   % Calculate S-G coefficients

HalfWin  = ((F+1)/2) -1;

%neck_pan_joint
%l_eye_version_joint
%l_eye_vergence_joint

msgs = readMessages(bagSelection);

initial_time = msgs{1,1}.Header.Stamp;


lastMsg = size(msgs);
lastMsg = lastMsg(1);

times = zeros(1, lastMsg);
neck_pan_velocity = zeros(1, lastMsg);
l_eye_version_velocity = zeros(1, lastMsg);
l_eye_vergence_velocity = zeros(1, lastMsg);


for i=1:lastMsg
    neck_pan_velocity(i) = msgs{i,1}.Velocity(8,1);
end


%Get all the times and all joint velocities

for i=1:lastMsg
    neck_pan_position(i) = msgs{i,1}.Position(8,1);
end

for i=(F+1)/2:lastMsg-(F+1)/2
    secs = (msgs{i,1}.Header.Stamp.Sec - initial_time.Sec);
    nsecs = (double(msgs{i,1}.Header.Stamp.Nsec)*10^-9 - double(initial_time.Nsec)*10^-9);
    times(i) = double(secs)+double(nsecs);
    
    
      % Zeroth derivative (smoothing only)
    SG0(i-HalfWin) = dot(g(:,1),neck_pan_position(i - HalfWin:i + HalfWin));
    
    
      % 1st differential
    SG1(i-HalfWin) = dot(g(:,2),neck_pan_position(i - HalfWin:i + HalfWin));
    
%    neck_pan_velocity(i) = msgs{i,1}.Position(8,1);         %Is the sign right?
%    l_eye_version_velocity(i) = -msgs{i,1}.Position(12,1);  %Is the sign right?
%    l_eye_vergence_velocity(i) = -msgs{i,1}.Position(16,1); %Is the sign right?
end

%Combined movement - http://wiki.icub.org/wiki/Vergence,_Version_and_Disparity

left_eye = l_eye_version_velocity+l_eye_vergence_velocity/.2;

figure(1);
hold on;

dt = mean(diff(times((F+1)/2:lastMsg-(F+1)/2)));

%plot(times((F+1)/2:lastMsg-(F+1)/2), SG0);
plot(times((F+1)/2:lastMsg-(F+1)/2), SG1/dt);
plot(times((F+1)/2:lastMsg-(F+1)/2), neck_pan_velocity((F+1)/2:lastMsg-(F+1)/2));

%plot(times((F+1)/2:lastMsg-(F+1)/2), neck_pan_position((F+1)/2:lastMsg-(F+1)/2));

legend('velocity - filter', 'velocity');

hold off;

end

