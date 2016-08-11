function el_angle = inverse_euler( euler_matrix )
% This function used to get the three Euler angle from the Euler
% Transformation Matrix.
% input: Euler Transformation Matrix.
% output: Three Euler angles;
% we make supposition as below:
% el_beta is in [0, pi], el_alpha and gamma is in [-pi, pi]
% The Euler transformation sees to the function 'euler_trans()'.
[m, n] = size(euler_matrix);
el_alpha = 0.0; el_beta = 0.0; el_gamma = 0.0;
if m < 3 || n < 3
    disp('This is not a normal Euler Matrix');
else
    if euler_matrix(3, 3) ~= 1
        el_beta = acos(euler_matrix(3, 3));
        el_alpha = acos(-euler_matrix(2, 3)/(1-sqrt(euler_matrix(3, 3))));
        el_gamma = acos(euler_matrix(3, 2)/(1-sqrt(euler_matrix(3, 3))));
        % here we suppose el_beta always be in [0, pi],
        % so when euler_matrix(1, 3) or (3, 1) is negative,
        % turn the alpha or gamma to [-pi, pi];
        if euler_matrix (1, 3) < 0
            el_alpha = -el_alpha;
        end
        if euler_matrix (3, 1) < 0
            el_gamma = -el_gamma;
        end
    else
        el_alpha = acos(euler_matrix(1, 1));
        el_beta = 0.0;
        el_gamma = 0.0;
    end
end
el_angle = [el_alpha; el_beta; el_gamma];
end