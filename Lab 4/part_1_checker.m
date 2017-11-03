%Sequence 1 (Projection) Checker
clear all;
close all;
clc;

load Sequence1Homographies

Image_00a = imread('SEQUENCE1/Image_00a.png');
Image_01a = imread('SEQUENCE1/Image_02a.png');

point_ref = [316 290 1];
point_check = Sequence1Homographies(1).H * point_ref';
point_check = point_check / point_check(end);

figure; imshow(Image_00a); impixelinfo; hold on;
plot(point_ref(1), point_ref(2), 'go');
figure; imshow(Image_01a); impixelinfo; hold on;
plot(point_check(1), point_check(2), 'go');

%% %Sequence 2 (Scaling) Checker
clear all;
close all;
clc;

load Sequence2Homographies

Image_00a = imread('SEQUENCE2/Image_00a.png');
Image_01a = imread('SEQUENCE2/Image_02a.png');

point_ref = [316 290 1];
point_check = Sequence2Homographies(1).H * point_ref';
point_check = point_check / point_check(end);

figure; imshow(Image_00a); impixelinfo; hold on;
plot(point_ref(1), point_ref(2), 'go');
figure; imshow(Image_01a); impixelinfo; hold on;
plot(point_check(1), point_check(2), 'go');

%% %Sequence 3 (Rotation) Checker

clear all;
close all;
clc;

load Sequence3Homographies

Image_00a = imread('SEQUENCE3/Image_00a.png');
Image_01a = imread('SEQUENCE3/Image_02a.png');

point_ref = [316 290 1];
point_check = Sequence3Homographies(1).H * point_ref';
point_check = point_check / point_check(end);

figure; imshow(Image_00a); impixelinfo; hold on;
plot(point_ref(1), point_ref(2), 'go');
figure; imshow(Image_01a); impixelinfo; hold on;
plot(point_check(1), point_check(2), 'go');

