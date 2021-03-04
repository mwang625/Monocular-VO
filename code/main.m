clear;
clc;
% Setup

ds = 2; % 0: KITTI, 1: Malaga, 2: parking

if ds == 0
    kitti_path = '../kitti00';
    % need to set kitti_path to folder containing "00" and "poses"
    assert(exist('kitti_path', 'var') ~= 0);
    ground_truth = load([kitti_path '/poses/00.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
    last_frame = 4540;
    K = [7.188560000000e+02 0 6.071928000000e+02
        0 7.188560000000e+02 1.852157000000e+02
        0 0 1];
elseif ds == 1
    % Path containing the many files of Malaga 7.
    malaga_path ='../malaga-urban-dataset-extract-07'
    assert(exist('malaga_path', 'var') ~= 0);
    images = dir([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images']);
    left_images = images(3:2:end);
    last_frame = length(left_images);
    K = [621.18428 0 404.0076
        0 621.18428 309.05989
        0 0 1];
elseif ds == 2
    % Path containing images, depths and all...
    parking_path = '../parking';
    assert(exist('parking_path', 'var') ~= 0);
    last_frame = 598;
    K = load([parking_path '/K.txt']);
     
    ground_truth = load([parking_path '/poses.txt']);
    ground_truth = ground_truth(:, [end-8 end]);
else
    assert(false);
end

% Bootstrap
% need to set bootstrap_frames
addpath('additional_functions/');
if ds == 0
    img0 = imread([kitti_path '/00/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(1))]);
    img1 = imread([kitti_path '/00/image_0/' ...
        sprintf('%06d.png',bootstrap_frames(2))]);
elseif ds == 1
    img0 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(1)+1).name]));
    img1 = rgb2gray(imread([malaga_path ...
        '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
        left_images(bootstrap_frames(2)).name]));
elseif ds == 2
    img0 = rgb2gray(imread([parking_path...
        sprintf('/images/img_%05d.png',bootstrap_frames(1))]));
    img1 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(2))]));
else
    assert(false);
end

%% key point detection, matching, triangulation
addpath('triangulation/triangulation/');
addpath('triangulation/8point/');
addpath('triangulation/');
addpath('triangulation/plot/');
addpath('bundle_adjustment/');

% rng(6);
kps0 = kps_detection(img0, ds);
[matched_pts0, matched_pts1, ~] = kps_matching(kps0, img0, img1); % ds for params tune; 2 by n [col row]
mpts0_h = padarray(matched_pts0, 1, 1, 'post'); %3 by n [col row 1]
mpts1_h = padarray(matched_pts1, 1, 1, 'post'); %3 by n [col row 1]
[R, t, inliers] = estimateRelativePose(mpts0_h, mpts1_h, K);
M0 = K * eye(3,4);
M1 = K * [R t];
P = linearTriangulation(mpts0_h(:,inliers), mpts1_h(:, inliers), M0, M1); % triangulate inliers only
matched_pts0 = matched_pts0(:,inliers);
matched_pts1 = matched_pts1(:,inliers);
% get inliers from the matched points and manually delete  points where z < 0
idx = P(3, :) > 0;
lm_tem = P(1:3, idx);
matched_pts0 = matched_pts0(:, idx);
matched_pts1 = matched_pts1(:, idx);
% delete where the z is larger than 15 times the median of z
idx = lm_tem(3, :) < 15 * median(lm_tem(3, :));
lm = lm_tem(1:3, idx);
matched_pts0 = matched_pts0(:, idx);
matched_pts1 = matched_pts1(:, idx);
% plot3(P(1,:), P(2,:), P(3,:), 'o');
% center_cam2_W = -R'*t;
% plotCoordinateFrame(R',center_cam2_W, 0.8);
% hold on;
% plot_3d(lm, R, t, img0, img1, matched_pts0, matched_pts1);
% plot initial two images and inlier outlier matches


%% Continuous operation
% for continuous plot
figure(6);
% subplot(1, 3, 3);
% lm_x = lm(1, :); lm_y = lm(2, :); lm_z = lm(3, :);
% lm_handle = scatter3(lm_x, lm_y, lm_z, 5);
% view(0,0);
% set(gcf, 'GraphicsSmoothing', 'on');
% axis equal;
% axis vis3d;
% axis([-10 50 0 30 -10 50]);

hold on;
% plot camera coordinate system
% plotCoordinateFrame(R', -R' * t, 3);
% view(0,0);
% set(gcf, 'GraphicsSmoothing', 'on');
% view(0,0);
% axis equal;
% axis vis3d;
% initial params
addpath('ransac_localization/');
range = (bootstrap_frames(2)+1):last_frame;
% create structure
lm_handle.XDataSource = 'lm_x'; lm_handle.YDataSource = 'lm_y'; lm_handle.ZDataSource = 'lm_z';
State.kpts = matched_pts1';
State.landmarks = lm;
prev_kps = matched_pts1';
prev_img = img1;
prev_lm = lm;
prev_R = eye(3);
prev_t = zeros(3,1);
keyframe = img1;
mpts_kf = prev_kps';
R0 = eye(3);
t0 =zeros(3,1);
T =[];
T2 = [];
T0 = [];
count = 0;
count2 = 0;
numinlier = [];
for i = range
    fprintf('\n\nProcessing frame %d\n=====================\n', i);
    if ds == 0
        image = imread([kitti_path '/00/image_0/' sprintf('%06d.png',i)]);
    elseif ds == 1
        image = rgb2gray(imread([malaga_path ...
            '/malaga-urban-dataset-extract-07_rectified_800x600_Images/' ...
            left_images(i).name]));
    elseif ds == 2
        image = im2uint8(rgb2gray(imread([parking_path ...
            sprintf('/images/img_%05d.png',i)])));
    else
        assert(false);
    end
    % Makes sure that plots refresh.   
    pause(0.01);
    % get the valid keypoints to estimate the pose
    [mpts_prev, mpts_current, isFound] = kps_matching(prev_kps, prev_img, image);
    prev_kps = mpts_current';
    
    lm_current = prev_lm(:, isFound);
    prev_lm = lm_current;
    
    [R, t, inlier_mask, ~] = ransacLocalization(mpts_current, lm_current, K);
    if sum(inlier_mask) > 80
    [R,t] = adjustp3p(R,t,lm_current(:,inlier_mask),mpts_current(:,inlier_mask),K);
    inlier_mask = refineInllier(R,t,mpts_current, lm_current, K);
    T0 = [T0,t];
    end
    if sum(inlier_mask) < 30
    [R,t] = adjustp3p(eye(3),median(T0,2),lm_current,mpts_current,K); 
    inlier_mask = refineInllier(R,t,mpts_current, lm_current, K);
    count2 = count2 +1;
    end
    numinlier = [numinlier, sum(inlier_mask)];
%     if sum(inlier_mask) < 30
%         R = eye(3);
%         t = t0;
%     end
    dist1 = median(lm_current(3,:));
    dist2 = norm(t);

    sum(inlier_mask)
    
    dist1
    dist2
    T = [T, -prev_R*R'*t + prev_t];
    T2 = [T2, -R'*t];
    if dist2 > 0.05*dist1 || sum(inlier_mask) < 80
        kps = kps_detection(image, ds);
        [mpts1, mpts2, isFound] = kps_matching(kps, image, keyframe); 

        pt1 = [mpts2;ones(1,length(mpts2))];
        pt2 = [mpts1;ones(1,length(mpts1))];

        M1 = K * eye(3,4);
        M2 = K * [R, t ];
        P = linearTriangulation(pt1,pt2,M1,M2);
        valid = P(3,:) > 0 & sum(abs( P(1:3,:) ) ) < 50*norm(t) & sum(abs( P(1:3,:) ) ) > norm(t);  

         P = R*P(1:3,:) + t; 

        prev_lm = P(1:3,valid);
        prev_t = prev_t - prev_R*R'*t;
        prev_R = prev_R*R';
        
        keyframe = image;
        prev_kps = mpts1(:,valid)';
        count = count + 1
    end
    
    prev_img = image;

    % plot the trajactory
    subplot(2, 2, 1);
    imshow(image);
    hold on;
    if (nnz(inlier_mask) > 0) && false
        plot(mpts_current(1, (inlier_mask)>0), ...
            mpts_current(2, (inlier_mask)>0), 'gx', 'Linewidth', 2);
        plot(mpts_prev(1, (inlier_mask)>0), ...
            mpts_prev(2, (inlier_mask)>0), 'rx', 'Linewidth', 2);
    end
    pause(0.01); 
    hold off;
    
    subplot(2, 2, 2);
    hold off
    n = i - bootstrap_frames(2) -1  ;
    if n < 10 
    plot( i-n+1:i,numinlier(end-n + 1:end) );
    else
    plot(i-9:i, numinlier(end-9:end) );
    end
    
    subplot(2, 2, 3);
    hold off
    
    plot(T(1,:),T(3,:),'r+')
    axis equal;
    
    subplot(2, 2, 4);
    hold off
    n = i - bootstrap_frames(2) -1  ;
    n2 = size(T,2);
    valid2 = P(3,:) > 0 & sum(abs( P(1:3,:) ) ) < 50*norm(t) & sum(abs( P(1:3,:) ) ) > norm(t);  
    Q = prev_R*P(1:3,:) + prev_t;
    Q = Q(:,valid2);
    plot(Q(1,:),Q(3,:), 'g.');
    hold on
    if n < 20 
    plot(T(1,n2-n+1:n2),T(3,n2-n+1:n2),'b.')
    else
    plot(T(1,i-19:n2),T(3,i-19:n2),'b.' );
    end
    axis equal;
    
    hold off

end
figure(6);
hold off;