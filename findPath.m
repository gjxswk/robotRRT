function [q_out, X_out, T_out] = findPath(q, X, parent, cost, T, ...
    endPoint, K)
% find path to goal in the current tree.

% input:
% q: the joint path in the tree
% X: the pose path in the tree
% parent: parent array of each point
% cost: cost of each point
% T: time spent of each point
% endPoint: array consisi of points that near the goal point
% K: the circulate times

% output:
% q_out|X_out|T_out: relative to the output path
min_cost = cost(endPoint(1));
best_endPoint = endPoint(1);
numEndPoint = size(endPoint);
% find the best end point first using the cost variable 
for i = 1:numEndPoint
    if cost(endPoint(i)) < min_cost
        min_cost = cost(endPoint(i));
        best_endPoint = i;
    end
end

n = size(q, 1);
m = size(X, 1);
q_out = zeros(n, K);
X_out = zeros(m, K);
T_out = zeros(K, 1);
i = best_endPoint;
j = 1;
while i ~= 0
    q_out(:, j) = q(:, i);
    X_out(:, j) = X(:, i);
    T_out(j) = T(i);
    i = parent(i);
    j = j + 1;
end
q_out = flipud(q_out(:, 1:j-1)')';
X_out = flipud(X_out(:, 1:j-1)')';
T_out = flipud(T_out(1: j-1));
end