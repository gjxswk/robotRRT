function res = goalTest(X_current, X_goal, eplsion)
% This function used to detect if current point reach the goal

% input:
% X_current: the current pose 
% X_goal: the goal pose
% eplsion: the given zero normalize

% output:
% res: 0 for not goal, and 1 the inverse.
if nargin < 3
    eplsion = 0.01;
end
res = 0;
if rrtDistance(X_current, X_goal) < eplsion
    res = 1;
end

end