function res = boundaryFree(q, q_max, q_min)
% boundary detection
% input variables:
% q: current 7 angle for each joint
% q_max/qmin: the maximun and minum angle boundary for machine

res=1;
n = length(q_max);
for i = 1:n
    if q(i) > q_max(i) || q(i) < q_min(i)
        res = 0;
    end
end
end