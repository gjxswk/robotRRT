function res = obstacleFree(Point, obstacle)
% This function used to detect if the current position of machine is in
% collision with the obstacle.
% input:
% Point: 3 x n matrix, where n is the number of points in Point
% obstacle: the obstacle matrix, there is different format for different
%   types of obstacle
% output:
% res: the detect result, 1 for free of obstacle, and 0 the other case.
res = 1;
safespace = 0.1;
numObstacle = size(obstacle, 1);
numPoint = size(Point, 2);
dimPoint = 3;
tig = zeros(numPoint, 1);
for k = 1:numObstacle
    if obstacle(k, 1) == 1
        % ºÏ≤‚∑Ω–Œ’œ∞≠ŒÔ
        for i = 1:numPoint
            for j = 1:dimPoint
                if abs(Point(j,i)-obstacle(k,j+1)) < obstacle(k,j+4)/2 ...
                    + safespace 
                    tig(j) = 1;
                else
                    tig(j) = 0;
                end
            end
            if min(tig) == 1
                res = 0;
                break;
            end
        end
    elseif obstacle(k,1)==2
        %ºÏ≤‚«Ú–Œ’œ∞≠ŒÔ
        for i = 1:numPoint
            if norm(Point(:,i)'-obstacle(k,2:4)) < abs(obstacle(k,5))
                res = 0;
                break;
            end
        end
    end
    if res == 0
        break;
    end

end