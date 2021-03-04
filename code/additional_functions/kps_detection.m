function kps_coords = kps_detection(img, ds)
% input: img matrix
% ouput: keypoints coords in the image, n by 2 matrix
mode = 2;

addpath('./harris_detector/');
if mode == 0
is_plot = false;
% defaut params for Kitti, 
harris_patch_size = 9;
harris_kappa = 0.08;
nonmaximum_supression_radius = 8;

% Other parameters.
num_keypoints = 2000;
if ds == 2 % parking 
    num_keypoints = 800;
    harris_patch_size = 7;
    harris_kappa = 0.07;
    nonmaximum_supression_radius = 12;
elseif ds == 1
    num_keypoints = 200;
    harris_patch_size = 9;
    harris_kappa = 0.08;
    nonmaximum_supression_radius = 8;
end

% Detect and match keypoints.
harris_scores = harris(img, harris_patch_size, harris_kappa);
kps = selectKeypoints(...
    harris_scores, num_keypoints, nonmaximum_supression_radius); % 2 by n, [row col]

kps_coords = fliplr(kps'); % change the row col in the matrix to ensure the correct order after transpose
end

if mode == 1
    points = detectSURFFeatures(img,'MetricThreshold',10);
%     kps_coords = double( fliplr(points.Location) );
    kps_coords = double( points.Location );
end

if mode == 2
     points = detectFASTFeatures(img,'MinContrast',0.01,'MinQuality',0.01);
%     kps_coords = double( fliplr(points.Location) );
    kps_coords = double( points.Location );
end

end

