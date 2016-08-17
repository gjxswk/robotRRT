function res = test_new_rap
% test the newton-raphson algorithm
res = 1;
test = load('RRT3.mat');
robot = load('robotDH.mat');
x_goal = [0.5; 0.5; 0.72; 2.300005618827138; 1.569999996710732; 
    -1.569999996458239];
[q_dest, x_dest, e_dest, suc, q_iter, x_iter] = newton_raphson(test.q0, x_goal, robot);
if suc
    toolkit('array', q_dest, 'q_dest: ');
    toolkit('array', x_dest, 'x_dest: ');
    toolkit('array', e_dest, 'e_dest: ');
    toolkit('matrix', q_iter, 'q_iter: ');
    toolkit('matrix', x_iter, 'x_iter: ');
end
end