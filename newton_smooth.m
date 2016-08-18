function [q_out, x_out, t_out, success, q_iter, x_iter, t_iter] ...
    = newton_smooth(q_in, x_goal, steps, obstacle, robot, time)
% This function use to do NP and test the path using smooth method.

% input:
% q_in: initial joint angles
% x_goal: the goal pose to go
% steps: the steps going to take on
% obstacle: postion matrix of obstacle
% robot: DH data of robot

% output:
% q_out|x_out|t_out: the output joint angles|pose|time taken
% success: 1 means success and 0 fail.
% q_iter|x_iter|t_iter: joint angles|pose|time in each iteration cycle
if nargin < 6
    time = 1;
end
t_out = time;
M = int32(steps);
q_iter = zeros(robot.n, M);
x_iter = zeros(robot.m, M);
t_iter = zeros(1, M);
[q_out, x_out, ~, success] = newton_raphson(q_in, x_goal, robot);
if success
     [q_iter, x_iter, t_iter, success] = smooth_path(q_in, q_out, steps,...
        obstacle, robot, time);
    t_out = t_iter(M);
end

end