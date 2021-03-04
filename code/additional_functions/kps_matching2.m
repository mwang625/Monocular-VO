function [matched_p0, matched_p1, isFound] = kps_matching2(kps0, img0, img1)
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
      

matched_p0 = matched_p0';
matched_p1 = matched_p1';
end