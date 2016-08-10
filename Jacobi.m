function [J, pose_each, R_accu, P_accu, w_accu, P_end] = Jacobi(q, robotOrDis, ...
a, alpha)
% This function is used to compute Jacobi matrix under the current state
% of robot.

% input variables:
% q: input 1x7 angle, for each joint
% robotOrDis: if the robot data is read from a structure(meaning .mat file),
%   then it is a structure input, the following input can be regardless. In
%   the other case, this is a 1x7 matrix for d in DH
% a: 1x7 matrix for a in DH
% alpha: 1x7 matrix for alpha in DH

% output variables:
% J: Jacobi matrix
% pose_each: (4x4xn matrix)pose expression of each joint, include R_each 
%   & P_each
% R_accu: rotate matrix for current z coordinate system in base coordinate
%   system, 3x3x(n+1) matrix. one position expanded
% P_accu: current O(0,0,0)'s coordinate in base coordiante system, 3x(n+1)
%   matrix, one position expanded
% w_accu: current z-axis's coordinate in base coordinate system, 3x(n+1)
%   matrix, one position expanded
% P_end: the posiontion for end point coordinate system in current
%   ordinate system, 3xn matrix

% Jocobi矩阵计算方法采用Whitney矢量计算方法，简单来说，J = [J1, .. , Jn], 对
% 于每一个Ji, 需要计算对（x,y,z,alpha,beta,gamma)六个变量的偏导，后三者其实
% 是坐标系O-XiYiZi的Zi轴在基坐标（O-X0Y0Z0)中的表示，前三者是末端速度在x,y,z轴
% 的三个分量，利用角速度矢量（即前边计算得到的）叉乘位置矢量（末端位置相对于当前
% 坐标系(O-XiYiZi)可得到

% attention!这里采用的建连杆坐标系方法是：基坐标系O-x0y0z0与theta_1重合，也就是
% 说，基坐标系自带一个自由变量（关节1的旋转角theta_1），然后O-x1y1z1建于关节2
% 上，往下类推，最后关节O-x7y7z7建于操作手末端。因此，计算Jacobi矩阵角速度列时
% 第一列为基坐标的角速度，应该为（0,0,1)，即z轴本身，之后依次乘上旋转矩阵，错开
% 一位。而计算位置矢量时，也是从基坐标开始，注意错开的顺序和矩阵的意义。为了使最
% 末端位置的角速度列纳入结果，故将R_accu、P_accu及w_accu扩充一位，但Jacobi矩阵
% 不需要该扩充位，注意。

% if input argument is 2, then read data from structure

if nargin == 2
    d = robotOrDis.d;
    a = robotOrDis.a;
    alpha = robotOrDis.alpha;
else
    d = robotOrDis;
end

% to enhance the use of this function, read n itself, not always .mat file
n = length(q);
m = 6;
J = zeros(m, n);
% pose_each is each pose matrix in Euler ZXZ form, the same with the other
% two
pose_each = zeros(4, 4, n); 
R_each = zeros(3, 3, n);
P_each = zeros(3, n);

for i = 1:n
    pose_each(:, :, i) = forward_kinematic(q(i), d(i), a(i), alpha(i), 1);
    R_each(:, :, i) = pose_each(1:3, 1:3, i);
    P_each(:, i) = pose_each(1:3, 4, i);
end

% to compute the accumulate pose matric, R for Euler angle and  
% P for position, n+1 is a expand bit under n joints.
R_accu = zeros(3, 3, n+1);
P_accu = zeros(3, n+1);
R_accu(:, :, 1) = eye(3);
P_accu(:, 1) = zeros(3, 1);
w_accu = zeros(3, n+1);
% compute angle velocity
for i = 2:n+1
    R_accu(:, :, i) = R_accu(:, :, i-1)*R_each(:, :, i-1); 
    P_accu(:, i) = P_accu(:, i-1) + R_accu(:, :, i-1) * P_each(:, i-1);
end
% the 1:n columns of w_accu matrix is the Jacobi angle velocity, also the 
% third column of R_accu matrix, that is the result [R_accu x (0, 0, 1)], 
% meaning the z-axis's expression in the base station.
w_accu(:, :) = R_accu(:, 3, :);
J(4:6, :) = w_accu(:, 1:n);
% compute velocity
% P_end is the relative coordinate in base coordinate system from current
% coordinate system to the end point coordinate system, and then using 
% v = w X r to compute velocity.
P_end = zeros(3, n);
for i = 1:n
   P_end(:, i) = P_accu(:, n+1) - P_accu(:, i);
   J(1:3, i) = cross(w_accu(:, i), P_end(:, i));
end

end
