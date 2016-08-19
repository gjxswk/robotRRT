function [q_out, x_out, t_out] = smooth(q_input, x_input, t_input)
% Smooth the path from initial point to goal point using algorithm method.

% input:
% q_input|x_input|t_input: the input joint path, pose path and time spent
%   of the given path
% output:
% q_out|x_out|t_out: the relative variables output
steps = length(t_input);
m = size(x_input, 1);
n = size(q_input, 1);
q_out = zeros(n, steps);
x_out = zeros(m, steps);
t_out = zeros(steps, 1);

q_out(:, 1) = q_input(:, 1);
x_out(:, 1) = x_input(:, 1);
t_out(1) = t_input(1);

delta_T = 3;
j = 2;
zero_norm = -0.001;
% delete the path that needs time less than zero_norm
for i = 2:steps
    if zero_norm < t_input(i) - delta_T*(j-1) 
        q_out(:, j) = q_input(:, i);
        x_out(:, j) = x_input(:, i);
        t_out(j) = t_input(i);
        j = j + 1;
    end
end
% add the last point into output result
q_out(:, j) = q_input(:, steps);
x_out(:, j) = x_input(:, steps);
t_out(j) = t_input(steps);
% delete redundant elements
q_out = q_out(:, 1:j);
x_out = x_out(:, 1:j);
t_out = t_out(1:j);
end