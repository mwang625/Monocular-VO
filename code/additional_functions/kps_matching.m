function [matched_p0, matched_p1, isFound] = kps_matching(kps0, img0, img1)
% doc:
% input: kps0 n by 2 
% output: two 2 by n matrices of the matched keypoints location, [x y] ->
% [col row]

tracker = vision.PointTracker('MaxBidirectionalError',2);
initialize(tracker, kps0, img0);
[kps1, isFound] = step(tracker, img1);
% find mathed points
matched_p0 = kps0(isFound, :);
matched_p1 = kps1(isFound, :);


% exculude outliers
outlier_excude = false;
if outlier_excude
    [~, matched_p0, matched_p1] = estimateGeometricTransform(...
                matched_p0, matched_p1, 'similarity', 'MaxDistance', 4);
end
        
% plot image and matched keypoint
plot = false;
if plot
    pointImage_0 = insertMarker(img0,matched_p0,'+','Color','white');
    pointImage_1 = insertMarker(img1,matched_p1,'+','Color','red');
    
    figure('Color', 'w');
    subplot(1, 2, [1 2]);
    imshow(pointImage_1);
    imshow(pointImage_0);

    hold on;
%     matched_p0(:, 1)
    scatter(matched_p0(:, 1)', matched_p0(:, 2)', 'gx');

    daspect([1 1 1]);

    axis off;
end
matched_p0 = matched_p0';
matched_p1 = matched_p1';
end