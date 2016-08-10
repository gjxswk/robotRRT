function rot_matrix = rot( angle, direction)
% function: use to perform rotation matrix in homogeneous way.
% input variables:
% distance: the distance to transform
% direction: char 'x' for x axis, the same with y and z axis.
% output: homogeneous rotation matrix
rot_matrix = eye(4);
if direction == 'x' 
    rot_matrix(2, 2) = cos(angle);
    rot_matrix(2, 3) = -sin(angle);
    rot_matrix(3, 2) = sin(angle);
    rot_matrix(3, 3) = cos(angle);
elseif direction == 'y'
    rot_matrix(1, 1) = cos(angle);
    rot_matrix(1, 3) = sin(angle);
    rot_matrix(3, 1) = -sin(angle);
    rot_matrix(3, 3) = cos(angle);
else
    rot_matrix(1, 1) = cos(angle);
    rot_matrix(1, 2) = -sin(angle);
    rot_matrix(2, 1) = sin(angle);
    rot_matrix(2, 2) = cos(angle);
end
end
