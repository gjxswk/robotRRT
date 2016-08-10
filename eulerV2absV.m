function [abs_v] = eulerV2absV(cur_angle, euler_v)
% This function translate the angle velocity in Euler form into angle 
% velocity in absolute coordinate system, which is always static.

% input variables:
% cur_angle: the current euler angles(psi, theta, phi)
% euler_v(v_psi, v_theta, v_phi): the three euler angles needed to compute

% output variables:
% abs_v(v_x, v_y, v_z): the three angle velocity output, which is the 
% current angle velocity relative to the base coordinate system.
psi = cur_angle(1); theta = cur_angle(2); % phi = cur_angle(3);
v_psi = euler_v(1); v_theta = euler_v(2); v_phi = euler_v(3);
euler2abs_matrix = [0, cos(psi), sin(psi)*sin(theta); 
                    0, sin(psi), -cos(psi)*sin(theta);
                    1,        0,           cos(theta)];
abs_v = euler2abs_matrix * [v_psi, v_theta, v_phi];
end