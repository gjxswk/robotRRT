function [q_out, x_out, error_out, success, q_iter, x_iter] = ...
    newton_raphson(q_in, x_dest, robot, error)
% perform the inverse kinematic algorithm, to get the road points in 
% joint space to destination point.

% input:
% q_in: the initial points in joint space, nx1 matrix
% x_dest: the dest point in pose space, mx1 matrix
% robot: the DH parameters of robot
% error: the error allowed for the last answer

% output:
% q_out|x_out: the outptu result in joint space|pose space
% error_out: the last error between x_out & x_dest
% success: return 1 if find out the answer, else stands for failed.
% q_iter|x_iter: each steps when compute in the iteration

% initialize
n = robot.n;
m = robot.m;
success = 1;
q_out = zeros(n, 1);
x_out = zeros(m, 1);
% error can be leave out and use the appointed value
if nargin < 4
    error = 1e-6;
end
% detect if jacobi matrix is singular
[jac, ~, r_in, p_in] = Jacobi(q_in, robot);
jtj = jac*jac';
det_jtj = det(jtj);
sin_error = (1e-3)^6;
if abs(det_jtj) < sin_error
    success = 0;
    disp('Error: In newton_raphson method, the Jacobi matrix is singular.');
    return
end
j_pinv = jac'/jtj; % if not singular, then continue to compute the pinv
x_in = matrix2pose(r_in(:, :, n+1), p_in(:, n+1));
delta_x = rrtDistance(x_in, x_dest);
dx = x_dest - x_in;
% begin iteration
i = 1;
i_max = 1000; % the maximum iteration times
q_iter = zeros(n, i_max);
q_iter(:, 1) = q_in(:);
x_iter = zeros(m, i_max);
x_iter(:, 1) = x_in(:);
while error < delta_x && i < i_max
    toolkit('matrix', j_pinv, 'j_pinv is: ');
    toolkit('matrix', jac, 'jac is: ');
    toolkit('matrix', q_iter(:, i), 'qi is: ');
    toolkit('matrix', dx, 'dx is: ');
    toolkit('matrix', delta_x, 'delta_x is: ');
    toolkit('matrix', x_iter(:, i), 'xi is: ');
    toolkit('input');
    q_iter(:, i+1) = q_iter(:, i) + j_pinv*dx; % newton-raphson
    % compute j_pinv
    [jac, ~, r_in, p_in] = Jacobi(q_iter(:, i+1), robot);
    jtj = jac*jac';
    det_jtj = det(jtj); % to see if jacobi is singular
    if abs(det_jtj) < sin_error
        success = 0;
        disp('Error: In newton_raphson method, Jacobi matrix singular.');
        return;
    end
    j_pinv = jac'/jtj;
    x_iter(:, i+1) = matrix2pose(r_in(:, :, n+1), p_in(:, n+1)); % x(i+1)
    delta_x = rrtDistance(x_iter(:, i), x_iter(:, i+1)); % correct delta_x
    dx = x_dest - x_iter(:, i+1);
    i = i + 1;
end
if delta_x < error
    q_out = q_iter(:, i);
    x_out = q_iter(:, i);
    q_iter = q_iter(:, 1:i);
    x_iter = x_iter(:, 1:i);
    error_out = delta_x;
else
    disp('Not success. The iteration times is out of the biggest range.');
    success = 0;
end
end