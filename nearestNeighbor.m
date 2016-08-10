function [point, rank, min_dis] = nearestNeighbor(X_rand, X, size)
% return the nearest neighbor point in the RRT tree to the current point
% appointed by X_rand.
% input variables:
% X_rand: a point in working space, often got by the sample function.
% X: the RRT tree
% size: the number of point in RRT tree
% output variables:
% point: return the point which is the nearest point to the given point.
% rank: the rank of point in RRT tree.
% min_dis: the RRT distance between point and given point
rank = 1;
min_dis = rrtDistance(X(:,1), X_rand);
point = X(:, 1);
for i = 2:size
    dis = rrtDistance(X(:, i), X_rand);
    if dis < min_dis
        min_dis = dis;
        rank = i;
        point = X(:, i);
    end
end
end