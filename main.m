function res = main()
% the main process entrance, performing a specific task

% define global variables here
global COMPILE; % use to generate compile information
COMPILE = 1;
% read in the given test data
test = load('RRT3.mat');
q0 = test.q0;
RE_goal = test.RE_goal;
rE_goal = test.rE_goal;
X_max = test.X_max;
X_min = test.X_min;
obstacle = test.obstacle;
% test RRT process and produce the path from q0 to goal
[q_path, x_path, t_path, succ] = RRT(q0, X_max, X_min, rE_goal, ...
    RE_goal, obstacle);
if ~succ
    if COMPILE
        disp('The process cannnot find the path');
    end
else
    if COMPILE
        disp('Find the path successfully.');
    end
    x_axis = 1:size(t_path);
    plot3(x_axis, t_path, q_path(:, 1), x_path(:, 1));
end
res = 1;
end
