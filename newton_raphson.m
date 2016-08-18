function [q_out, x_out, error_out, success, q_iter, x_iter, error_iter] ...
    = newton_raphson(q_in, x_dest, robot, error)
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
error_out = 1e10;
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
R_in = r_in(:, :, n+1); % the current euler transformation matrix(ETM)
R_dest = euler_trans(x_dest(4), x_dest(5), x_dest(6));
R_dest = R_dest(1:3, 1:3);% destination ETM
x_in = matrix2pose(R_in, p_in(:, n+1));
delta_x = rrtDistance(x_in, x_dest);
dx = zeros(m, 1);
dx(1:3) = x_dest(1:3) - x_in(1:3);
dx(4:6) = rotate_fix_axis(R_in, R_dest);
% begin iteration
i = 1;
i_max = 10; % the maximum iteration times
q_iter = zeros(n, i_max);
q_iter(:, 1) = q_in(:);
x_iter = zeros(m, i_max);
x_iter(:, 1) = x_in(:);
error_iter = zeros(1, i_max);
error_iter(:, 1) = abs(delta_x);
while error < abs(delta_x) && i < i_max
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
    R_in = r_in(:, :, n+1);
    x_iter(:, i+1) = matrix2pose(R_in, p_in(:, n+1)); % x(i+1)
    delta_x = rrtDistance(x_iter(:, i+1), x_dest); % correct delta_x
    error_iter(:, i+1) = delta_x; % compute error in each iteration
    dx(1:3) = x_dest(1:3) - x_iter(1:3, i+1); % dx, dy, dz
    dx(4:6) = rotate_fix_axis(R_in, R_dest); % fix-axis rotation
    i = i + 1;
end
if abs(delta_x) < error
    q_out = q_iter(:, i);
    x_out = x_iter(:, i);
    q_iter = q_iter(:, 1:i);
    x_iter = x_iter(:, 1:i);
    error_iter = error_iter(:, 1:i);
    error_out = delta_x;
else
    % find the best answer as result
    for k = 1:i
        [min_err, rank] = min(error_iter(:));
        error_out = min_err;
        q_out = q_iter(:, rank);
        x_out = x_iter(:, rank);
    end
    success = 0;
end
end