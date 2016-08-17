function [rela_angle, rela_matrix] = rotate_fix_axis(angleA, angleB)
% compute the relative angle from coordinate A to coordinate B system, and
% both of their current system is relative to the base coordinate system.
% So what we compute is the rotate angle & fix-axis in base coordinate
% from A system to B System

% input:
% angleA|angleB: the two input parameters, they can be euler angle or 3x3
%   matrix of euler transformation matrix.

% output:
% rela_angle: the output rotate angle's three part diretion(x, y, z) vector
% rela_matrix: the output 3 relative transformation matrix
[m, n] = size(angleA);
if nargin == 1
    if (m == 3 && n == 1) || (m == 1 && n == 3)
        rela_matrix = euler_trans(angleA(1), angleA(2), angleA(3));
    elseif m == 3 && n == 3
        rela_matrix = angleA;
    else
        disp('In relative_euler, input format is not valified.');
        return;
    end
else
    if (m == 3 && n == 1) || (m == 1 && n == 3)
        matrixA = euler_trans(angleA(1), angleA(2), angleA(3));
        matrixB = euler_trans(angleB(1), angleB(2), angleB(3));
    elseif m == 3 && n == 3
        matrixA = angleA;
        matrixB = angleB;
    else
        disp('In relative_euler, input format is not valified.');
        return;
    end
    rela_matrix = matrixB * matrixA';
end
rot_angle = acos((rela_matrix(1,1)+rela_matrix(2,2)+rela_matrix(3,3)-1)/2);
ki(1) = 1/(2*sin(rot_angle))*(rela_matrix(3,2) - rela_matrix(2,3));
ki(2) = 1/(2*sin(rot_angle))*(rela_matrix(1,3) - rela_matrix(3,1));
ki(3) = 1/(2*sin(rot_angle))*(rela_matrix(2,1) - rela_matrix(1,2));
rela_angle = [ki(1), ki(2), ki(3)] * rot_angle;
end