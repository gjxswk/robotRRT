function grad = gradMLG(q, q_max, q_min, n)
% compute the gradient of H in the appointment expression

% input:
% q: curent angle
% q_max & q_min: the maximun and minimun of angle that can be input
% n: length of q_max, q_min & q

% output:
% grad: the gradient output
if nargin == 3
    n = length(q_max);
end
grad = zeros(n,1);
L = zeros(n,1);
omiga = pi;
for i = 1:n
    if q_max(i,1) - q(i,1) < omiga || q(i,1) - q_min(i,1) < omiga
        a = (q_max(i,1) - q(i,1)) * (q(i,1) - q_min(i,1));
        b = 2*q(i,1) - q_max(i,1) - q_min(i,1);
        c = q_max(i,1) - q_min(i,1);
        grad(i,1) = c^2*b/(4*a^2);
        L(i,1) = 1/2*c^2/a^2*(1 + b^2/a);
    end
end
G = diag(L);
grad = (grad'*grad)/(grad'*G*grad)*grad;
end