function [ velocity ] = computeVelocities( jointPosition, N, F, HalfWin, dt)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

[b,g] = sgolay(N,F);   % Calculate S-G coefficients

for i=(F+1)/2:lastMsg-(F+1)/2
    % Zeroth derivative (smoothing only)
    SG0(i-HalfWin) = dot(g(:,1),jointPosition(i - HalfWin:i + HalfWin));
    
    % 1st differential
    SG1(i-HalfWin) = dot(g(:,2),jointPosition(i - HalfWin:i + HalfWin));
end

if dt == 0
    disp('Warning: dt is zero!');
end

velocity = SG1/dt;



end

