function trans_matrix = trans( distance, direction )
% function: use to perform transformation matrix in homogeneous way.
% input variables:
% distance: the distance to transform
% direction: char 'x' for x axis, the same with y and z axis.
% output: homogeneous transformation matrix
trans_matrix = eye(4);
if direction == 'x'
    trans_matrix (1, 4) = distance;
elseif direction == 'y'
    trans_matrix (2, 4) = distance;
else
    trans_matrix (3, 4) = distance;
end

end