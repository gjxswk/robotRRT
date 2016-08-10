function euler_matrix = euler_trans( el_alpha, el_beta, el_gamma )

% use to perform Euler transfomation, with the three given angle, which
% is also the pose repersentation in this work.
% input: the three Euler angle
% output: 3x3 euler trans_matrix, written in homogeneous form.
% discription: The Euler transform is doing in this way.
% 1. We rotate with the Z-axis to el_alpha degree; 
% 2. rotate with the newly X-axis to el_beta degree
% 3. rotate with the newly Z-axis to el_gamma degree

euler_matrix = rot(el_alpha, 'z') * rot(el_beta, 'x') * rot(el_gamma, 'z');

end