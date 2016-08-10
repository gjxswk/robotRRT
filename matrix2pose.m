function [position, euler_angle] = matrix2pose( homo_matrix, ...
    pos_matrix )
% to get the pose representation from the homogeneous matrix,
% all of then obey the euler z-x-z rotate rules.
% input: the homogeneous matrix and pos_matrix(x, y, z), if there are
% 2 arguments input, then the first one is to be 3x3 Euler_matrix, else
% the first one is 4x4 Euler homogeneous matrix.
% output: (x, y, z) is the position, (alpha, beta, gamma) is for the 
% three Euler angle.

if nargin < 2
    position(1:3) = homo_matrix(1:3, 4);
    euler_angle(1:3) = inverse_euler(homo_matrix);
else
    position(1:3) = pos_matrix(1:3);
    euler_angle(1:3) = inverse_euler(homo_matrix);
end 
% if only one output, then write the euler angle information into position
% matrix to get one matrix contain all the information
if nargout < 2
    position(4:6) = euler_angle(1:3);
end
end