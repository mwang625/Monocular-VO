function [R, t, inlier_mask, j] = ransacLocalizationRept(mpts_current, lm_current, K)
[R, t, inlier_mask, ~] = ransacLocalization(mpts_current, lm_current, K);
j = 0;
while j < 5 & sum(inlier_mask) < 100
    [R, t, inlier_mask, ~] = ransacLocalization(mpts_current, lm_current, K);
    j = j + 1;
end

end