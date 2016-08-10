function res = toolkit(name, text)
% a tool for compiling or other use.
syms x y z psi theta phi
rot_x(x) = [1      0       0;
            0 cos(x) -sin(x);
            0 sin(x) cos(x)];
rot_y(x) = [cos(x)  0  sin(x);
            0       1       0;
            -sin(x) 0 cos(x)];
rot_z(x) = [cos(x) -sin(x) 0;
            sin(x)  cos(x) 0;
                 0       0 1];
euler = rot_z(x) * rot_x(x) * rot_z(x);
res = 1;
end