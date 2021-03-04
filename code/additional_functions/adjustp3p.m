
function [R,t] = adjustp3p(R0,t0,P,p,K)
% Returns the points of the estimated trajectory p_V_C transformed into the
% ground truth frame G. The similarity transform Sim_G_V is to be chosen
% such that it results in the lowest error between the aligned trajectory
% points p_G_C and the points of the ground truth trajectory pp_G_C. All
% matrices are 3xN.

% Initial guess is identity.
T = [R0,t0];
T = [T;0,0,0,1];
x = HomogMatrix2twist(T);

% Using an external error function for nicer code. Binding pp_G_C and p_V_C
% by casting the function as a function of the hidden state only.
error_terms = @(x) alignError32(x, P, p, K);
options = optimoptions(@lsqnonlin, 'Display', 'off');
x_optim = lsqnonlin(error_terms, x, [], [], options);

T= twist2HomogMatrix(x_optim);
R = T(1:3,1:3);
t = T(1:3,4);


end


