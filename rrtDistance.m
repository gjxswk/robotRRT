function dis = rrtDistance(pa, pb)
% the distance function use in this RRT function, which can be change to 
% adapt to the real use of course.
% input variables:
% pa & pb: two point in workspace, 1x6 matrix each.
alpha = 0.8;
dis = alpha*norm(pa(1:3)-pb(1:3), 2) + (1-alpha)*norm(pa(4:6)-pb(4:6), 2);
end