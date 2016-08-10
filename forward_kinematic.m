function [ end_pose ] = forward_kinematic(theta, robotOrDis, a, alpha,...
    n_Joints)
% this function used to do forward kinematic for robot.
% input variables:
% n_Joints: the joints number, also the length of a, alpha, d & theta
% a: 连杆偏移量数组
% alpha: 连杆扭转角度数组
% d: 连杆长度
% theta: 关节角
% output variables:
% end_pos : 4x4 matrix for end pose in homogeneous transformation matrix
% 正向解步骤：
% 1. 首先让当前坐标系x轴x_n与下一个坐标系x轴x_n+1平行，故绕z_n（当前的z轴，以下类
% 推）旋转theta_n角度
% 2. 其次，为使x_n和x_n+1轴共线，将x_n轴沿z_n轴平移d_n
% 3. 为使x_n和x_n+1轴共原点，将x_n轴沿自身平移a_n
% 4. 为使z_n和z_n+1轴重合，绕x_n轴旋转alpha_a
% 5. 依关节数量累乘得齐次变换正解矩阵
if nargin < 4 % 表示从结构读入变量数据
    d = robotOrDis.d;
    a = robotOrDis.a;
    alpha = robotOrDis.alpha;
    n_Joints = robotOrDis.n;
elseif nargin < 5  % 缺省变量（关节数）
    d = robotOrDis.d;
    a = robotOrDis.a;
    alpha = robotOrDis.alpha;
    n_Joints = length(theta);
else
    d = robotOrDis;
end

end_pose = eye(4); 
for i = 1 : n_Joints
    end_pose = end_pose * rot(theta(i), 'z') * trans(d(i), 'z') ...
    * trans(a(i), 'x') * rot(alpha(i), 'x');
end

end

