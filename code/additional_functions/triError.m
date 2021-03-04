function error = triError(P, p1,p2, M1,M2)
% Given x which encodes the similarity transform Sim_G_V as a concatenation
% of twist and scale, return the error pp_G_C - p_G_C (p_G_C = Sim_G_V * 
% p_V_C) as a single column vector.

q1 = M1*[P;1];
q2 = M2*[P;1];

errors = [p1 - q1/q1(3); p2 - q2/q2(3)] ;

error = errors(:);

end