function [ times neck_pan_velocity  l_eye_version_velocity l_eye_vergene_velocity] = plotNeckEyesVels( bagObject )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here


%Select the joint states topic

bagSelection = select(bagObject, 'Topic', '/joint_states');

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


%Get all the times and all joint velocities
for i=1:lastMsg
    secs = (msgs{i,1}.Header.Stamp.Sec - initial_time.Sec);
    nsecs = (double(msgs{i,1}.Header.Stamp.Nsec)*10^-9 - double(initial_time.Nsec)*10^-9);
    times(i) = double(secs)+double(nsecs);
    neck_pan_velocity(i) = msgs{i,1}.Velocity(8,1);         %Is the sign right?
    l_eye_version_velocity(i) = -msgs{i,1}.Velocity(12,1);  %Is the sign right?
    l_eye_vergence_velocity(i) = -msgs{i,1}.Velocity(16,1); %Is the sign right?
end

%Combined movement - http://wiki.icub.org/wiki/Vergence,_Version_and_Disparity

left_eye = l_eye_version_velocity+l_eye_vergence_velocity/.2;

figure(1);
hold on;

plot(times, neck_pan_velocity);
plot(times, left_eye);
%plot(times, l_eye_vergence_velocity);

legend('Neck Pan Velocity', 'Left eye velocity');

hold off;

end

