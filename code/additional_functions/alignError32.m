
function error = alignError32(x, P, p, K)
% Given x which encodes the similarity transform Sim_G_V as a concatenation
% of twist and scale, return the error pp_G_C - p_G_C (p_G_C = Sim_G_V * 
% p_V_C) as a single column vector.

T_G_V = twist2HomogMatrix(x(1:6));
% scale_G_V = x(7);
R = T_G_V(1:3, 1:3);
t = T_G_V(1:3, 4);
p1 = K*(R*P + t);
errors = p - p1(1:2,:)./p1(3,:);

error = errors(:);

