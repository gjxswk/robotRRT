function [q_path, X_free, parent, cost, Time, success] = extendNP(...
    q_path, X_free, parent, cost, Time, X_rand, rank, factor, obstacle, ...
    robot)
% This function perform the RRT method 'extend' use in newton-raphsson
% method.

% input variables:
% q_path: the current free space for joint orbit
% X_free: the current free space for pose orbit
% parent: the father point of each point, take down by rank(in free space)
% cost: the cost function, usually the distance(in free space)
% Time: the time taken to go to each point(in free space)
% X_rand: the random point in space, usually got by method sample()
% rank: rank of current big clcle in RRT while, also the size of extended
%   point of q
% factor: a factor, deciding how many steps should be done in one MLG 
%   cycle(2*factor), and deciding the extending distance(factor / 100)
% obstacle: the obstacle variable
% robot: the structure of robot DH method and so on

% output variables:
% q_path|X_free}parent|cost|Time: after extend the new point into the old tree, 
%   all of this variables are refreshed.
% success: 1 for extending successfully, and 0 for fail.

% define global variables
global COMPILE
COMPILE = 1;
% 找出X_rand最近点X_free(i)
[near_p, near_rank, near_dis] = nearestNeighbor(X_rand, X_free, rank-1);
if COMPILE
    toolkit('array', near_p, 'near point is: ');
    toolkit('array', near_rank, 'near rank is: ');
    toolkit('array', near_dis, 'near distance is: ');
end
% if near enough, then use the rand point as new point, else perform
% linear rule.
ext_dis = 0.01*factor;
X_new = produceNewPoint(near_p, X_rand, ext_dis, near_dis);
if COMPILE
    toolkit('array', X_new, 'new point is: ');
end
% use the new point to perform newton-raphson
q_initial = q_path(:, near_rank);
cost_min = min(ext_dis, near_dis);
steps = double(int8(2*factor));
% doing MLG, get the joint orbit from q_near to q_new
[q_out, x_out, ~, success] = newton_raphson(q_initial, X_new, ...
    robot);
% if success, extend the new point to the orginal tree
if success
    [~, ~, t_iter, success] = smooth_path(q_initial, q_out, steps,...
        obstacle, robot);
    q_path(:, rank) = q_out;
    X_free(:, rank) = x_out;
    Time(rank) = Time(near_rank) + t_iter(steps);
    cost(rank) = cost(near_rank) + cost_min;
    parent(rank) = near_rank;
end
end