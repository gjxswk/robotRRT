function [q_iter, x_iter, t_iter, success] = smooth_path(q_in, q_dest, ...
    steps, obstacle, robot, time)
% doing smooth work and avoid obstacle
% input:

% output:

% leave out 'time' parameters
if nargin < 6
    time = 1.0;
end
success = 1;
if time == 0
    disp('In smooth_path method, time cannot be 0.');
    return;
end
num = int32(steps);
t = zeros(1, num);
for i = 2:num
    t(i) = double(i) / double(num);
end
t_iter = t*time;
a_5 = 6*(q_dest - q_in)*(t.^5);
a_4 = -15*(q_dest - q_in)*(t.^4);
a_3 = 10*(q_dest - q_in)*(t.^3);
a_2 = zeros(robot.n, num);
a_1 = zeros(robot.n, num);
a_0 = q_in*ones(1, num);
q_iter = a_5 + a_4 + a_3 + a_2 + a_1 + a_0;
x_iter = zeros(robot.m, num);
for i = 1:num
    pose = forward_kinematic(q_iter(:, i), robot);
    x_iter(:, i) = matrix2pose(pose);
    if ~(boundaryFree(q_iter(:, i), robot.q_max, robot.q_min) && ...
            obstacleFree(x_iter, obstacle)) 
        success = 0;
        break;
    end
end
end