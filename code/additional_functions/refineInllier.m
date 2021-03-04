function inlier = refineInllier(R,t,p, P, K)
p1 = K*(R*P + t);
errors = sum( abs( p - p1(1:2,:)./p1(3,:) ) );
inlier = errors < 20;



end