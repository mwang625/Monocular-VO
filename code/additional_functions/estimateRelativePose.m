function [R,t, inliers] = estimateRelativePose(pts0, pts1, K)
% doc:
% input:
% -pts0, pts1: 3 by n coords [col row]
% -K: intrinsic params
% output: R, t
is_normalize = true;
% % normalization points if not included in the following function
if is_normalize
    [p1_nh,T1] = normalise2dpts(pts0);
    [p2_nh,T2] = normalise2dpts(pts1);
    % extract E, 
    [F, inliers] = estimateFundamentalMatrix(p1_nh(1:2, :)', p2_nh(1:2, :)', ...
        'Method','RANSAC', 'NumTrials', 8000, 'DistanceThreshold', 1e-4);
    fprintf('number of inliers %d.\n', sum(inliers));
    F = (T2.') * F * T1;
    E = K'* F * K;
else
    % extract E, 
    [F, inliers] = estimateFundamentalMatrix(pts0(1:2, :)', pts1(1:2, :)', ...
        'Method','RANSAC', 'NumTrials', 3000, 'DistanceThreshold', 1e-4);
    fprintf('number of inliers %d.\n', sum(inliers));
    E = K'* F * K;
end

% extract R and t, relative pose
[Rs,u3] = decomposeEssentialMatrix(E);
[R, t] = disambiguateRelativePose(Rs, u3, pts0, ...
                                    pts1, K, K); % change pts0 to [row col]
end
