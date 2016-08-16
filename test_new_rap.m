function res = test_new_rap
% test the newton-raphson algorithm
res = 1;
test = load('RRT3.mat');
robot = load('robotDH.mat');
x_goal = [0.5; 0.5; 0.72; 2.3; 1.57; -1.57];
[q_dest, x_dest, e_dest, suc] = newton_raphson(test.q0, x_goal, robot);
if suc
    toolkit('array', q_dest, 'q_dest: ');
    toolkit('array', x_dest, 'x_dest: ');
    toolkit('array', e_dest, 'e_dest: ');
end
end