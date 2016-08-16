function [newPoint] = produceNewPoint(nearPoint, randPoint, ext_dis, ...
    near_dis)
% This function use to produce new point, and mainly use in RRT extend use

% input:
% nearPoint & randPoint: the nearest point to randPoint in current tree, 
%   and the randPoint is produce randmonly using sample() method.
% ext_dis & near_dis: ext_dis means one step distance, and near_dis means
%   the current distance between nearPoint & randPoint
% output:
% newPoint: produce a new point that is suitable to go down
global COMPILE;
if nargin < 4
    near_dis = rrtDistance(randPoint, nearPoint);
end
if ext_dis < near_dis
    if COMPILE
        disp('In function produce new point, ext_dis < near_dis.');
    end
    newPoint = nearPoint + ext_dis/near_dis*(randPoint - nearPoint);
else
    if COMPILE
        disp('In function produce new point, near_dis < ext_dis');
    end
    newPoint = randPoint;
end

end