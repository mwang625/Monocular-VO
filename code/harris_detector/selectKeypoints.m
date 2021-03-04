function keypoints = selectKeypoints(scores, num, r)
% Selects the num best scores as keypoints and performs non-maximum 
% supression of a (2r + 1)*(2r + 1) box around the current maximum.

keypoints = zeros(2, num);
% keypoints = [];
temp_scores = padarray(scores, [r r]);
for i = 1:num
    [a, kp] = max(temp_scores(:));
% 
%     if a == 0
%         
%         break

    [row, col] = ind2sub(size(temp_scores), kp);
    kp = [row;col];
    keypoints(:, i) = kp - r;
%     keypoints = [keypoints, kp - r];
    temp_scores(kp(1)-r:kp(1)+r, kp(2)-r:kp(2)+r) = ...
        zeros(2*r + 1, 2*r + 1);
end
end

