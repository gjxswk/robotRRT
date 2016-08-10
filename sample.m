function point = sample(leftLimit, rightLimit)
% use this function to generate a ramdom point in workSpace to perform
% RRT sample.
% input variable:
% workSpace's left and right limit: means the working space in which the 
% ramdom point is going to generate.
% output variable:
% point: return the sample point, format is decided by the workSpace, that
%   is to say, if in pose space, then point will be a pose(1x6 matrix), if
%   in angle space, then point will probably be an (1x7 matrix) angle
%   matrix.
m = length(leftLimit);
point = leftLimit + diag(rightLimit-leftLimit) * rand([m, 1]);

end