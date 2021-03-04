

parking_path = '../data_parking';
I1 = rgb2gray(imread([parking_path...
        sprintf('/images/img_%05d.png',bootstrap_frames(1))]));
I2 = rgb2gray(imread([parking_path ...
        sprintf('/images/img_%05d.png',bootstrap_frames(2))]));

points1 = detectFASTFeatures(I1,'MinContrast',0.01,'MinQuality',0.01);
points2 = detectFASTFeatures(I2,'MinContrast',0.01,'MinQuality',0.01);

[f1,vpts1] = extractFeatures(I1,points1);
[f2,vpts2] = extractFeatures(I2,points2);

indexPairs = matchFeatures(f1,f2) ;
matchedPoints1 = vpts1(indexPairs(:,1));
matchedPoints2 = vpts2(indexPairs(:,2));

figure; showMatchedFeatures(I1,I2,matchedPoints1,matchedPoints2);